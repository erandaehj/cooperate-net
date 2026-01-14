package main

import (
	"encoding/json"
	"fmt"
	"math"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/hyperledger/fabric-contract-api-go/contractapi"
)

const (
	// Fixed-point bases
	MM  = int64(1000) // 1 m = 1000 mm
	MS  = int64(1000) // 1 s = 1000 ms
	EPS = int64(1)    // tiny epsilon for division guards

	// TTC cap (ms)
	TTC_CAP_MS = int64(30000)

	// Weight scale (permille)
	W_SCALE = int64(1000)

	// Normalization scale for J output (micro)
	J_SCALE = int64(1_000_000)
)

var MODE_ORDER = []string{"LV_only", "LV_plus_FDV", "LV_plus_LDV", "all_three"}

// ---------------------- Data Models ---------------------- //

type Vehicle struct {
	X0mm  int64 `json:"x0_mm"`
	V0mmS int64 `json:"v0_mms"`
}

type Params struct {
	// Timing
	Tms  int64 `json:"T_ms"`  // e.g., 4000
	Dtms int64 `json:"dt_ms"` // e.g., 100

	// Safety geometry
	Ellmm   int64 `json:"ell_mm"`   // 6000
	D0mm    int64 `json:"d0_mm"`    // 2000
	TauMs   int64 `json:"tau_ms"`   // 1000
	TTCmin  int64 `json:"ttc_min"`  // 2000
	EpsVmmS int64 `json:"eps_v"`    // 1
	Kappa   int64 `json:"kappa"`    // TTC penalty weight scale (integer)

	// Speed window
	Vmin int64 `json:"v_min_mms"` // 60 km/h ~ 16667
	Vmax int64 `json:"v_max_mms"` // 120 km/h ~ 33333

	// Efficiency
	Drefmm int64 `json:"d_ref_mm"` // 1500m => 1_500_000

	// Weights (permille), should sum to 1000 (or close)
	WSafety  int64 `json:"w_safety"`
	WComfort int64 `json:"w_comfort"`
	WEff     int64 `json:"w_eff"`

	// Comfort weights (integers)
	LamV     int64 `json:"lam_v"`
	LamFront int64 `json:"lam_front"`
	LamRear  int64 `json:"lam_rear"`

	// Efficiency weights (integers)
	EtaV     int64 `json:"eta_v"`
	EtaFront int64 `json:"eta_front"`
	EtaRear  int64 `json:"eta_rear"`

	// Pairwise safety sharing (permille follower share, fixed 0.5 => 500)
	FollowerShareGap int64 `json:"follower_share_gap"`
	FollowerShareTTC int64 `json:"follower_share_ttc"`
}

type Scenario struct {
	ID    string  `json:"id"`
	Front Vehicle `json:"front"` // LDV (front)
	LV    Vehicle `json:"lv"`
	Rear  Vehicle `json:"rear"` // FDV (rear)
	P     Params  `json:"params"`
	AtUTC string  `json:"created_utc"`
}

type ModeResult struct {
	ScenarioID string `json:"scenario_id"`
	Mode       string `json:"mode"`

	Success bool   `json:"success"`
	Status  string `json:"status"`

	AV int64 `json:"a_v_mmps2"`
	AF int64 `json:"a_front_mmps2"`
	AR int64 `json:"a_rear_mmps2"`

	// Raw totals
	Safety     int64 `json:"safety"`
	Comfort    int64 `json:"comfort"`
	Efficiency int64 `json:"efficiency"`

	// Normalized components (permille)
	SNorm int64 `json:"s_norm"`
	CNorm int64 `json:"c_norm"`
	ENorm int64 `json:"e_norm"`

	// J scaled (micro)
	Jscaled int64 `json:"j_scaled"`

	// Metrics
	MinGapFrontMM int64 `json:"min_gap_front_mm"`
	MinGapRearMM  int64 `json:"min_gap_rear_mm"`
	MinTTCFrontMS int64 `json:"min_ttc_front_ms"`
	MinTTCRearMS  int64 `json:"min_ttc_rear_ms"`

	// Terminal constraint margins (mm)
	GFrontLVT int64 `json:"g_front_lv_T"`
	GLVRearT  int64 `json:"g_lv_rear_T"`

	// Scales
	Sscale int64 `json:"s_scale"`
	Cscale int64 `json:"c_scale"`
	Escale int64 `json:"e_scale"`
}

type Summary struct {
	ScenarioID string `json:"scenario_id"`
	BestMode   string `json:"best_mode"`
	BestJ      int64  `json:"best_j_scaled"`
	MaxModeJ   int64  `json:"max_mode_j_scaled"`
	AllOK      bool   `json:"all_modes_success"`
}

// ---------------------- Contract ---------------------- //

type GridSolveContract struct {
	contractapi.Contract
}

// Default params consistent with your Python (but fixed-point)
func DefaultParams() Params {
	return Params{
		Tms:  4000,
		Dtms: 100,

		Ellmm:   6000,
		D0mm:    2000,
		TauMs:   1000,
		TTCmin:  2000,
		EpsVmmS: 1,
		Kappa:   1, // keep 1; scales handle magnitude

		Vmin: 16667,
		Vmax: 33333,

		Drefmm: 1500 * MM,

		WSafety:  333,
		WComfort: 333,
		WEff:     334,

		LamV:     1,
		LamFront: 1,
		LamRear:  1,

		EtaV:     1,
		EtaFront: 1,
		EtaRear:  1,

		FollowerShareGap: 500,
		FollowerShareTTC: 500,
	}
}

func (c *GridSolveContract) CreateScenario(ctx contractapi.TransactionContextInterface, id string, scenarioJSON string) error {
	if strings.TrimSpace(id) == "" {
		return fmt.Errorf("id must not be empty")
	}
	key := "SCENARIO_" + id

	exists, err := ctx.GetStub().GetState(key)
	if err != nil {
		return err
	}
	if exists != nil {
		return fmt.Errorf("scenario already exists: %s", id)
	}

	var s Scenario
	if strings.TrimSpace(scenarioJSON) == "" {
		// If empty, create a dummy with defaults (not recommended)
		s = Scenario{
			ID:    id,
			Front: Vehicle{X0mm: 0, V0mmS: 25000},
			LV:    Vehicle{X0mm: 12000, V0mmS: 25000},
			Rear:  Vehicle{X0mm: -12000, V0mmS: 25000},
			P:     DefaultParams(),
			AtUTC: time.Now().UTC().Format(time.RFC3339),
		}
	} else {
		if err := json.Unmarshal([]byte(scenarioJSON), &s); err != nil {
			return fmt.Errorf("invalid scenario JSON: %w", err)
		}
		s.ID = id
		if s.P.Tms == 0 {
			s.P = DefaultParams()
		}
		if s.AtUTC == "" {
			s.AtUTC = time.Now().UTC().Format(time.RFC3339)
		}
	}

	b, _ := json.Marshal(s)
	return ctx.GetStub().PutState(key, b)
}

func (c *GridSolveContract) SolveAllModesGrid(ctx contractapi.TransactionContextInterface, scenarioID string, steps int) (string, error) {
	if steps < 2 {
		steps = 2
	}
	s, err := loadScenario(ctx, scenarioID)
	if err != nil {
		return "", err
	}

	results := make(map[string]ModeResult, 4)
	for _, mode := range MODE_ORDER {
		r := solveModeGrid(s.Front, s.LV, s.Rear, s.P, mode, steps)
		results[mode] = r
		if err := putResult(ctx, scenarioID, mode, r); err != nil {
			return "", err
		}
	}

	// Summary
	bestMode := "none"
	bestJ := int64(math.MaxInt64)
	maxJ := int64(0)
	allOK := true
	for _, mode := range MODE_ORDER {
		r := results[mode]
		if !r.Success {
			allOK = false
		}
		if r.Jscaled > maxJ {
			maxJ = r.Jscaled
		}
		if r.Success && r.Jscaled < bestJ {
			bestJ = r.Jscaled
			bestMode = mode
		}
	}
	if bestMode == "none" {
		bestJ = -1
	}

	sum := Summary{
		ScenarioID: scenarioID,
		BestMode:   bestMode,
		BestJ:      bestJ,
		MaxModeJ:   maxJ,
		AllOK:      allOK,
	}
	if err := putSummary(ctx, scenarioID, sum); err != nil {
		return "", err
	}

	out, _ := json.Marshal(map[string]any{
		"scenario_id": scenarioID,
		"summary":     sum,
		"results":     results,
	})
	return string(out), nil
}

func (c *GridSolveContract) ReadScenario(ctx contractapi.TransactionContextInterface, id string) (string, error) {
	s, err := loadScenario(ctx, id)
	if err != nil {
		return "", err
		b, _ := json.Marshal(s)
	return string(b), nil
}

func (c *GridSolveContract) ReadResult(ctx contractapi.TransactionContextInterface, scenarioID string, mode string) (string, error) {
	key := "RESULT_" + scenarioID + "_" + mode
	b, err := ctx.GetStub().GetState(key)
	if err != nil {
		return "", err
	}
	if b == nil {
		return "", fmt.Errorf("result not found: %s %s", scenarioID, mode)
	}
	return string(b), nil
}

func (c *GridSolveContract) ReadSummary(ctx contractapi.TransactionContextInterface, scenarioID string) (string, error) {
	key := "SUMMARY_" + scenarioID
	b, err := ctx.GetStub().GetState(key)
	if err != nil {
		return "", err
	}
	if b == nil {
		return "", fmt.Errorf("summary not found: %s", scenarioID)
	}
	return string(b), nil
}

// ---------------------- Ledger helpers ---------------------- //

func loadScenario(ctx contractapi.TransactionContextInterface, id string) (Scenario, error) {
	key := "SCENARIO_" + id
	b, err := ctx.GetStub().GetState(key)
	if err != nil {
		return Scenario{}, err
	}
	if b == nil {
		return Scenario{}, fmt.Errorf("scenario not found: %s", id)
	}
	var s Scenario
	if err := json.Unmarshal(b, &s); err != nil {
		return Scenario{}, err
	}
	return s, nil
}

func putResult(ctx contractapi.TransactionContextInterface, scenarioID, mode string, r ModeResult) error {
	key := "RESULT_" + scenarioID + "_" + mode
	b, _ := json.Marshal(r)
	return ctx.GetStub().PutState(key, b)
}

func putSummary(ctx contractapi.TransactionContextInterface, scenarioID string, s Summary) error {
	key := "SUMMARY_" + scenarioID
	b, _ := json.Marshal(s)
	return ctx.GetStub().PutState(key, b)
}

// ---------------------- Deterministic grid solve core ---------------------- //

type bounds3 struct {
	Vlo, Vhi int64
	Flo, Fhi int64
	Rlo, Rhi int64
}

func accelBoundsFromSpeed(v0, vmin, vmax, Tms int64) (int64, int64) {
	// a = 1000*(v_target - v0)/Tms  in mm/s^2 (because v is mm/s)
	lo := (MS * (vmin - v0)) / Tms
	hi := (MS * (vmax - v0)) / Tms
	if lo > hi {
		lo, hi = hi, lo
	}
	return lo, hi
}

func speedBasedBounds(front, lv, rear Vehicle, P Params, mode string) bounds3 {
	vlo, vhi := accelBoundsFromSpeed(lv.V0mmS, P.Vmin, P.Vmax, P.Tms)
	flo, fhi := accelBoundsFromSpeed(front.V0mmS, P.Vmin, P.Vmax, P.Tms)
	rlo, rhi := accelBoundsFromSpeed(rear.V0mmS, P.Vmin, P.Vmax, P.Tms)

	switch mode {
	case "LV_only":
		return bounds3{Vlo: vlo, Vhi: vhi, Flo: 0, Fhi: 0, Rlo: 0, Rhi: 0}
	case "LV_plus_FDV":
		return bounds3{Vlo: vlo, Vhi: vhi, Flo: 0, Fhi: 0, Rlo: rlo, Rhi: rhi}
	case "LV_plus_LDV":
		return bounds3{Vlo: vlo, Vhi: vhi, Flo: flo, Fhi: fhi, Rlo: 0, Rhi: 0}
	case "all_three":
		return bounds3{Vlo: vlo, Vhi: vhi, Flo: flo, Fhi: fhi, Rlo: rlo, Rhi: rhi}
	default:
		// fallback
		return bounds3{Vlo: vlo, Vhi: vhi, Flo: flo, Fhi: fhi, Rlo: rlo, Rhi: rhi}
	}
}

func linspaceI64(lo, hi int64, steps int) []int64 {
	if steps <= 1 || hi <= lo {
		return []int64{lo}
	}
	out := make([]int64, steps)
	den := int64(steps - 1)
	for i := 0; i < steps; i++ {
		num := int64(i)
		out[i] = lo + ((hi-lo)*num)/den
	}
	return out
}

func clampI64(x, lo, hi int64) int64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}

// kinematics at time t_ms
func kinematicsAt(x0, v0, a int64, t_ms int64) (x, v int64) {
	v = v0 + (a*t_ms)/MS
	if v < 0 {
		v = 0
	}
	term1 := (v0 * t_ms) / MS
	term2 := (a * t_ms * t_ms) / (2 * MS * MS)
	x = x0 + term1 + term2
	return x, v
}

func gap(frontX, rearX, ell int64) int64 {
	return frontX - rearX - ell
}

func dSafe(d0, tau_ms, v_follow int64) int64 {
	return d0 + (tau_ms*v_follow)/MS
}

func ttcMs(frontX, rearX, frontV, rearV, ell, epsV int64, capMs int64) int64 {
	ds := frontX - rearX - ell // mm
	dv := rearV - frontV       // mm/s
	if dv <= epsV {
		return capMs
	}
	if ds <= 0 {
		return 0
	}
	return (MS * ds) / dv // ms
}

func timeMsAt(Drefmm, vmmS int64) int64 {
	if vmmS < EPS {
		vmmS = EPS
	}
	return (MS * Drefmm) / vmmS
}

func terminalConstraints(front, lv, rear Vehicle, P Params, aF, aV, aR int64) (g1, g2 int64) {
	xF, _ := kinematicsAt(front.X0mm, front.V0mmS, aF, P.Tms)
	xV, vV := kinematicsAt(lv.X0mm, lv.V0mmS, aV, P.Tms)
	xR, vR := kinematicsAt(rear.X0mm, rear.V0mmS, aR, P.Tms)

	g1 = gap(xF, xV, P.Ellmm) - dSafe(P.D0mm, P.TauMs, vV)
	g2 = gap(xV, xR, P.Ellmm) - dSafe(P.D0mm, P.TauMs, vR)
	return
}

func pairShares(defaultFollowerSharePermille int64) (leaderShare, followerShare int64) {
	fs := clampI64(defaultFollowerSharePermille, 0, 1000)
	return 1000 - fs, fs
}

// Safety cost per vehicle split, but returns total + metrics only (for speed).
func safetyCostTotal(front, lv, rear Vehicle, P Params, aF, aV, aR int64) (S int64, minGapF, minGapR, minTTCF, minTTCR int64) {
	minGapF = int64(math.MaxInt64)
	minGapR = int64(math.MaxInt64)
	minTTCF = int64(math.MaxInt64)
	minTTCR = int64(math.MaxInt64)

	// fixed shares
	lfG, ffG := pairShares(P.FollowerShareGap)
	lfT, ffT := pairShares(P.FollowerShareTTC)
	_ = lfG
	_ = ffG
	_ = lfT
	_ = ffT

	var penTotal int64 = 0

	N := int(P.Tms / P.Dtms)
	for k := 0; k <= N; k++ {
		tms := int64(k) * P.Dtms

		xF, vF := kinematicsAt(front.X0mm, front.V0mmS, aF, tms)
		xV, vV := kinematicsAt(lv.X0mm, lv.V0mmS, aV, tms)
		xR, vR := kinematicsAt(rear.X0mm, rear.V0mmS, aR, tms)

		gF := gap(xF, xV, P.Ellmm)
		gR := gap(xV, xR, P.Ellmm)

		if gF < minGapF {
			minGapF = gF
		}
		if gR < minGapR {
			minGapR = gR
		}

		dsF := dSafe(P.D0mm, P.TauMs, vV)
		dsR := dSafe(P.D0mm, P.TauMs, vR)

		ttcF := ttcMs(xF, xV, vF, vV, P.Ellmm, P.EpsVmmS, TTC_CAP_MS)
		ttcR := ttcMs(xV, xR, vV, vR, P.Ellmm, P.EpsVmmS, TTC_CAP_MS)

		if ttcF < minTTCF {
			minTTCF = ttcF
		}
		if ttcR < minTTCR {
			minTTCR = ttcR
		}

		var step int64 = 0

		// gap shortfalls (mm^2)
		if dsF > gF {
			d := dsF - gF
			step += d * d
		}
		if dsR > gR {
			d := dsR - gR
			step += d * d
		}

		// TTC shortfalls (ms^2) weighted by Kappa
		if P.TTCmin > ttcF {
			d := P.TTCmin - ttcF
			step += P.Kappa * d * d
		}
		if P.TTCmin > ttcR {
			d := P.TTCmin - ttcR
			step += P.Kappa * d * d
		}

		// integrate: step * dt (ms). Keep raw magnitude; normalization handles it.
		penTotal += step * P.Dtms
	}
	return penTotal, minGapF, minGapR, minTTCF, minTTCR
}

func comfortCost(front, lv, rear Vehicle, P Params, aF, aV, aR int64) int64 {
	// a^2 in (mm/s^2)^2. Keep raw.
	return P.LamV*aV*aV + P.LamFront*aF*aF + P.LamRear*aR*aR
}

func efficiencyCost(front, lv, rear Vehicle, P Params, aF, aV, aR int64) int64 {
	// Time-loss only at terminal speed.
	_, vFT := kinematicsAt(front.X0mm, front.V0mmS, aF, P.Tms)
	_, vVT := kinematicsAt(lv.X0mm, lv.V0mmS, aV, P.Tms)
	_, vRT := kinematicsAt(rear.X0mm, rear.V0mmS, aR, P.Tms)

	// clamp to speed window
	vFT = clampI64(vFT, P.Vmin, P.Vmax)
	vVT = clampI64(vVT, P.Vmin, P.Vmax)
	vRT = clampI64(vRT, P.Vmin, P.Vmax)

	// desired speeds: initial speeds (as in your fallback)
	vFstar := front.V0mmS
	vVstar := lv.V0mmS
	vRstar := rear.V0mmS

	tF := timeMsAt(P.Drefmm, vFT)
	tV := timeMsAt(P.Drefmm, vVT)
	tR := timeMsAt(P.Drefmm, vRT)

	tFstar := timeMsAt(P.Drefmm, clampI64(vFstar, P.Vmin, P.Vmax))
	tVstar := timeMsAt(P.Drefmm, clampI64(vVstar, P.Vmin, P.Vmax))
	tRstar := timeMsAt(P.Drefmm, clampI64(vRstar, P.Vmin, P.Vmax))

	var loss int64 = 0
	if tV > tVstar {
		d := tV - tVstar
		loss += P.EtaV * d * d
	}
	if tF > tFstar {
		d := tF - tFstar
		loss += P.EtaFront * d * d
	}
	if tR > tRstar {
		d := tR - tRstar
		loss += P.EtaRear * d * d
	}
	return loss
}

func scalesFromBounds(front, lv, rear Vehicle, P Params, b bounds3) (Sscale, Cscale, Escale int64) {
	// Safety scale: max at 8 corners
	cornersV := []int64{b.Vlo, b.Vhi}
	cornersF := []int64{b.Flo, b.Fhi}
	cornersR := []int64{b.Rlo, b.Rhi}

	var smax int64 = 0
	for _, aV := range cornersV {
		for _, aF := range cornersF {
			for _, aR := range cornersR {
				S, _, _, _, _ := safetyCostTotal(front, lv, rear, P, aF, aV, aR)
				if S > smax {
					smax = S
				}
			}
		}
	}
	if smax < 1 {
		smax = 1
	}

	// Comfort scale: use max abs bounds
	aabsV := maxAbs(b.Vlo, b.Vhi)
	aabsF := maxAbs(b.Flo, b.Fhi)
	aabsR := maxAbs(b.Rlo, b.Rhi)
	C := P.LamV*aabsV*aabsV + P.LamFront*aabsF*aabsF + P.LamRear*aabsR*aabsR
	if C < 1 {
		C = 1
	}

	// Efficiency scale: worst case at minimum accel (slowest)
	E := efficiencyCost(front, lv, rear, P, b.Flo, b.Vlo, b.Rlo)
	if E < 1 {
		E = 1
	}
	return smax, C, E
}

func maxAbs(a, b int64) int64 {
	if a < 0 {
		a = -a
	}
	if b < 0 {
		b = -b
	}
	if a > b {
		return a
	}
	return b
}

// normalized permille: (val/scale)*1000
func normPermille(val, scale int64) int64 {
	if scale < 1 {
		scale = 1
	}
	// protect overflow with int64: val can be large; do (val/scale)*1000 first when possible
	// but keep precision: (1000*val)/scale
	return (W_SCALE * val) / scale
}

func solveModeGrid(front, lv, rear Vehicle, P Params, mode string, steps int) ModeResult {
	b := speedBasedBounds(front, lv, rear, P, mode)
	Sscale, Cscale, Escale := scalesFromBounds(front, lv, rear, P, b)

	gridV := linspaceI64(b.Vlo, b.Vhi, steps)
	gridF := linspaceI64(b.Flo, b.Fhi, steps)
	gridR := linspaceI64(b.Rlo, b.Rhi, steps)

	best := ModeResult{
		ScenarioID: "",
		Mode:       mode,
		Success:    false,
		Status:     "no feasible point",
		Sscale:     Sscale,
		Cscale:     Cscale,
		Escale:     Escale,
		Jscaled:    math.MaxInt64,
	}

	type triple struct{ aV, aF, aR int64 }
	cands := make([]triple, 0, len(gridV)*len(gridF)*len(gridR))
	for _, aV := range gridV {
		for _, aF := range gridF {
			for _, aR := range gridR {
				cands = append(cands, triple{aV: aV, aF: aF, aR: aR})
			}
		}
	}

	// Ensure deterministic iteration order even if slices somehow vary
	sort.Slice(cands, func(i, j int) bool {
		if cands[i].aV != cands[j].aV {
			return cands[i].aV < cands[j].aV
		}
		if cands[i].aF != cands[j].aF {
			return cands[i].aF < cands[j].aF
		}
		return cands[i].aR < cands[j].aR
	})

	for _, c := range cands {
		aV, aF, aR := c.aV, c.aF, c.aR

		// terminal constraints
		g1, g2 := terminalConstraints(front, lv, rear, P, aF, aV, aR)
		if g1 < 0 || g2 < 0 {
			continue
		}

		// terminal speeds within window (safety)
		_, vFT := kinematicsAt(front.X0mm, front.V0mmS, aF, P.Tms)
		_, vVT := kinematicsAt(lv.X0mm, lv.V0mmS, aV, P.Tms)
		_, vRT := kinematicsAt(rear.X0mm, rear.V0mmS, aR, P.Tms)
		if vFT < P.Vmin || vFT > P.Vmax || vVT < P.Vmin || vVT > P.Vmax || vRT < P.Vmin || vRT > P.Vmax {
			continue
		}

		S, minGapF, minGapR, minTTCF, minTTCR := safetyCostTotal(front, lv, rear, P, aF, aV, aR)
		C := comfortCost(front, lv, rear, P, aF, aV, aR)
		E := efficiencyCost(front, lv, rear, P, aF, aV, aR)

		Sn := normPermille(S, Sscale)
		Cn := normPermille(C, Cscale)
		En := normPermille(E, Escale)

		// J permille = (wS*Sn + wC*Cn + wE*En)/1000
		Jperm := (P.WSafety*Sn + P.WComfort*Cn + P.WEff*En) / W_SCALE
		Jscaled := Jperm * (J_SCALE / W_SCALE) // permille -> micro

		if Jscaled < best.Jscaled {
			best = ModeResult{
				ScenarioID:     "",
				Mode:           mode,
				Success:        true,
				Status:         "OK",
				AV:             aV,
				AF:             aF,
				AR:             aR,
				Safety:         S,
				Comfort:        C,
				Efficiency:     E,
				SNorm:          Sn,
				CNorm:          Cn,
				ENorm:          En,
				Jscaled:        Jscaled,
				MinGapFrontMM:  minGapF,
				MinGapRearMM:   minGapR,
				MinTTCFrontMS:  minTTCF,
				MinTTCRearMS:   minTTCR,
				GFrontLVT:      g1,
				GLVRearT:       g2,
				Sscale:         Sscale,
				Cscale:         Cscale,
				Escale:         Escale,
			}
		}
	}

	return best
}

// ---------------------- Main ---------------------- //

func main() {
	cc, err := contractapi.NewChaincode(new(GridSolveContract))
	if err != nil {
		panic(err.Error())
	}
	if err := cc.Start(); err != nil {
		panic(err.Error())
	}
}

// ---------------------- Utility: (optional) km/h to mm/s for client-side ---------------------- //
// Not used on-chain, but kept here as reference.
func kmhToMmps(kmh float64) int64 {
	mps := kmh * (1000.0 / 3600.0)
	return int64(math.Round(mps * 1000.0))
}

// ---------------------- Utility: parse helpers if you later add CLI-like args ---------------------- //
func parseI64(s string) (int64, error) { return strconv.ParseInt(strings.TrimSpace(s), 10, 64) }
