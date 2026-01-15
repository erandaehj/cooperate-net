'use strict';

const { WorkloadModuleBase } = require('@hyperledger/caliper-core');

function randBetween(rng, lo, hi) {
  return lo + rng() * (hi - lo);
}

function makeMulberry32(seed) {
  let t = seed >>> 0;
  return function() {
    t += 0x6D2B79F5;
    let x = t;
    x = Math.imul(x ^ (x >>> 15), x | 1);
    x ^= x + Math.imul(x ^ (x >>> 7), x | 61);
    return ((x ^ (x >>> 14)) >>> 0) / 4294967296;
  };
}

class GridSolveWorkload extends WorkloadModuleBase {
  constructor() {
    super();
    this.txIndex = 0;
  }

  async initializeWorkloadModule(workerIndex, totalWorkers, roundIndex, roundArguments, sutAdapter, sutContext) {
    await super.initializeWorkloadModule(workerIndex, totalWorkers, roundIndex, roundArguments, sutAdapter, sutContext);

    this.workerIndex = workerIndex;
    this.roundArgs = roundArguments;

    // Chaincode & channel
    this.contractId = roundArguments.contractId || 'overtakingcc';
    this.channel = roundArguments.channel || 'mychannel';

    // Steps for grid
    this.steps = Number(roundArguments.steps || 7);

    // Default params (must match chaincode DefaultParams unless you pass your own)
    this.params = {
      T_ms: 4000, dt_ms: 100,
      ell_mm: 6000, d0_mm: 2000, tau_ms: 1000,
      ttc_min: 2000, eps_v: 1, kappa: 1,
      v_min_mms: 16667, v_max_mms: 33333,
      d_ref_mm: 1500000,
      w_safety: 333, w_comfort: 333, w_eff: 334,
      lam_v: 1, lam_front: 1, lam_rear: 1,
      eta_v: 1, eta_front: 1, eta_rear: 1,
      follower_share_gap: 500, follower_share_ttc: 500
    };

    // RNG seed per worker
    const baseSeed = Number(roundArguments.seed || 0);
    this.rng = makeMulberry32(baseSeed + workerIndex * 9973);
  }

  // Scenario generator equivalent to your Python random_scenario()
  buildScenario(id) {
    const P = this.params;

    const vMin = P.v_min_mms;
    const vMax = P.v_max_mms;

    const vR0 = Math.floor(randBetween(this.rng, vMin, vMax));
    const vV0 = Math.floor(randBetween(this.rng, vMin, vMax));
    const vF0 = Math.floor(randBetween(this.rng, vMin, vMax));

    const gR0 = Math.floor(randBetween(this.rng, 2000, 30000)); // 2..30m in mm
    const gF0 = Math.floor(randBetween(this.rng, 2000, 30000));

    const xR0 = Math.floor(randBetween(this.rng, -5000, 5000)); // -5..5m in mm
    const xV0 = xR0 + P.ell_mm + gR0;
    const xF0 = xV0 + P.ell_mm + gF0;

    return {
      id,
      front: { x0_mm: xF0, v0_mms: vF0 },
      lv:    { x0_mm: xV0, v0_mms: vV0 },
      rear:  { x0_mm: xR0, v0_mms: vR0 },
      params: P
    };
  }

  async submitTransaction() {
    this.txIndex++;
    const scenarioId = `SCN_${this.workerIndex}_${this.txIndex}_${Date.now()}`;

    const scenario = this.buildScenario(scenarioId);

    // 1) CreateScenario
    const req1 = {
      contractId: this.contractId,
      channel: this.channel,
      contractFunction: 'CreateScenario',
      invokerIdentity: this.roundArgs.invoker || 'client0.org1.example.com',
      contractArguments: [scenarioId, JSON.stringify(scenario)],
      readOnly: false
    };
    await this.sutAdapter.sendRequests(req1);

    // 2) SolveAllModesGrid
    const req2 = {
      contractId: this.contractId,
      channel: this.channel,
      contractFunction: 'SolveAllModesGrid',
      invokerIdentity: this.roundArgs.invoker || 'client0.org1.example.com',
      contractArguments: [scenarioId, String(this.steps)],
      readOnly: false
    };
    await this.sutAdapter.sendRequests(req2);
  }
}

function createWorkloadModule() {
  return new GridSolveWorkload();
}

module.exports.createWorkloadModule = createWorkloadModule;
