// Sokoban solver — make.fun sokoban-solver-01. Push-optimal A* over pushes with room decomposition (P* is the sum of the generator's room optima), matching heuristic on push distances, dead-square + freeze/2x2 deadlock pruning, player-region canonicalisation. Budgets count expansions, never seconds; no clock is read, so executions are byte-identical.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const LET = ['u', 'd', 'l', 'r']; // 0=up 1=down 2=left 3=right

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  const nbr = new Int32Array(w * h * 4).fill(-1);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const i = y * w + x;
    if (y > 0) nbr[i * 4] = i - w;
    if (y < h - 1) nbr[i * 4 + 1] = i + w;
    if (x > 0) nbr[i * 4 + 2] = i - 1;
    if (x < w - 1) nbr[i * 4 + 3] = i + 1;
  }
  return { w, h, wall, goals, boxes, player, nbr };
}

function pushTables(L) {
  const { wall, goals, nbr } = L;
  const n = L.w * L.h;
  const tables = goals.map((g) => {
    const dist = new Int32Array(n).fill(-1);
    dist[g] = 0;
    const q = [g];
    for (let head = 0; head < q.length; head++) {
      const c = q[head];
      for (let dd = 0; dd < 4; dd++) {
        const b = nbr[c * 4 + (dd ^ 1)];
        if (b < 0 || wall[b] || dist[b] >= 0) continue;
        const stand = nbr[b * 4 + (dd ^ 1)];
        if (stand < 0 || wall[stand]) continue;
        dist[b] = dist[c] + 1;
        q.push(b);
      }
    }
    return dist;
  });
  const dead = new Uint8Array(n).fill(1);
  for (let i = 0; i < n; i++) {
    if (wall[i]) continue;
    for (const t of tables) if (t[i] >= 0) { dead[i] = 0; break; }
  }
  return { tables, dead };
}

function hungarian(cost, n) {
  const u = new Float64Array(n + 1);
  const v = new Float64Array(n + 1);
  const p = new Int32Array(n + 1).fill(-1);
  const way = new Int32Array(n + 1).fill(-1);
  for (let i = 0; i < n; i++) {
    p[n] = i;
    let j0 = n;
    const minv = new Float64Array(n + 1).fill(Infinity);
    const used = new Uint8Array(n + 1);
    do {
      used[j0] = 1;
      const i0 = p[j0];
      let delta = Infinity;
      let j1 = -1;
      for (let j = 0; j < n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 * n + j] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; } else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] !== -1);
    do {
      const j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 !== n);
  }
  const matchOf = new Int32Array(n).fill(-1);
  for (let j = 0; j < n; j++) if (p[j] >= 0) matchOf[p[j]] = j;
  let total = 0;
  for (let i = 0; i < n; i++) total += cost[i * n + matchOf[i]];
  return total;
}

function solvePush(L, T, boxCells, walkCells, boxes0, goalsSub, player0, budget) {
  const { wall, nbr } = L;
  const nB = boxes0.length;
  const goalSet = new Set(goalsSub);
  const tabs = goalsSub.map((g) => T.tables[L.goals.indexOf(g)]);
  const costBuf = new Float64Array(nB * nB);
  const heuristic = (boxes) => {
    if (nB <= 8 && tabs.length === nB) {
      for (let i = 0; i < nB; i++) for (let j = 0; j < nB; j++) {
        const d = tabs[j][boxes[i]];
        costBuf[i * nB + j] = d < 0 ? 1e6 : d;
      }
      return hungarian(costBuf, nB);
    }
    let s = 0;
    for (let i = 0; i < nB; i++) {
      let m = Infinity;
      for (let j = 0; j < nB; j++) { const d = tabs[j][boxes[i]]; if (d >= 0 && d < m) m = d; }
      s += m === Infinity ? 1e6 : m;
    }
    return s;
  };
  const seenArr = new Int32Array(L.w * L.h);
  let stamp = 0;
  const flood = (start, boxSet) => {
    stamp++;
    const stack = [start];
    seenArr[start] = stamp;
    let min = start;
    while (stack.length) {
      const c = stack.pop();
      if (c < min) min = c;
      for (let d = 0; d < 4; d++) {
        const q = nbr[c * 4 + d];
        if (q < 0 || seenArr[q] === stamp || wall[q] || !walkCells[q] || boxSet.has(q)) continue;
        seenArr[q] = stamp;
        stack.push(q);
      }
    }
    return min;
  };
  const keyOf = (boxes, pCanon) => String.fromCharCode(pCanon, ...boxes);
  const frozen = (b, boxSet, seenSet) => {
    if (seenSet.has(b)) return true;
    seenSet.add(b);
    const side = (d) => {
      const q = nbr[b * 4 + d];
      if (q < 0 || wall[q]) return true;
      if (!boxSet.has(q)) return false;
      return frozen(q, boxSet, seenSet);
    };
    return (side(0) || side(1)) && (side(2) || side(3));
  };
  const isDead = (boxes, boxSet) => {
    for (const b of boxes) {
      if (goalSet.has(b)) continue;
      const bx = b % L.w;
      const by = (b / L.w) | 0;
      for (let oy = -1; oy <= 0; oy++) for (let ox = -1; ox <= 0; ox++) {
        const x0 = bx + ox;
        const y0 = by + oy;
        if (x0 < 0 || y0 < 0 || x0 + 1 >= L.w || y0 + 1 >= L.h) continue;
        let allSolid = true;
        let anyOffGoal = false;
        for (let dy = 0; dy < 2 && allSolid; dy++) for (let dx = 0; dx < 2; dx++) {
          const c = (y0 + dy) * L.w + (x0 + dx);
          if (wall[c]) continue;
          if (boxSet.has(c)) { if (!goalSet.has(c)) anyOffGoal = true; continue; }
          allSolid = false;
        }
        if (allSolid && anyOffGoal) return true;
      }
      if (frozen(b, boxSet, new Set())) return true;
    }
    return false;
  };
  const startSet = new Set(boxes0);
  if (isDead(boxes0, startSet)) return null;
  const startCanon = flood(player0, startSet);
  const states = [{ boxes: boxes0.slice(), canon: startCanon, g: 0, parent: -1, box: -1, dir: -1 }];
  const best = new Map([[keyOf(boxes0, startCanon), 0]]);
  const heap = [[heuristic(boxes0), 0, 0, 0]];
  let counter = 0;
  let expansions = 0;
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e) => {
    heap.push(e);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const hpop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };
  while (heap.length) {
    const si = hpop()[3];
    const st = states[si];
    if (best.get(keyOf(st.boxes, st.canon)) !== si) continue;
    if (st.boxes.every((b) => goalSet.has(b))) {
      const seq = [];
      for (let t = si; t > 0; t = states[t].parent) seq.push({ box: states[t].box, dir: states[t].dir });
      seq.reverse();
      return { pushes: seq, nodes: expansions };
    }
    if (++expansions > budget) return null;
    const boxSet = new Set(st.boxes);
    stamp++;
    const reach = new Set();
    const stack = [st.canon];
    seenArr[st.canon] = stamp;
    while (stack.length) {
      const c = stack.pop();
      reach.add(c);
      for (let d = 0; d < 4; d++) {
        const q = nbr[c * 4 + d];
        if (q < 0 || seenArr[q] === stamp || wall[q] || !walkCells[q] || boxSet.has(q)) continue;
        seenArr[q] = stamp;
        stack.push(q);
      }
    }
    for (let bi = 0; bi < nB; bi++) {
      const b = st.boxes[bi];
      for (let d = 0; d < 4; d++) {
        const stand = nbr[b * 4 + (d ^ 1)];
        const dest = nbr[b * 4 + d];
        if (stand < 0 || dest < 0) continue;
        if (!reach.has(stand)) continue;
        if (wall[dest] || boxSet.has(dest) || !boxCells[dest] || T.dead[dest]) continue;
        const nb = st.boxes.slice();
        nb[bi] = dest;
        nb.sort((x, y) => x - y);
        const nSet = new Set(nb);
        if (isDead(nb, nSet)) continue;
        const nCanon = flood(b, nSet);
        const k = keyOf(nb, nCanon);
        const g = st.g + 1;
        const prev = best.get(k);
        if (prev !== undefined && states[prev].g <= g) continue;
        const ni = states.length;
        states.push({ boxes: nb, canon: nCanon, g, parent: si, box: b, dir: d });
        best.set(k, ni);
        hpush([g + heuristic(nb), -g, counter++, ni]);
      }
    }
  }
  return null;
}

function expandMoves(L, boxes0, player0, pushes) {
  const { wall, nbr } = L;
  const boxes = new Set(boxes0);
  let player = player0;
  let out = '';
  const walkTo = (target) => {
    const prev = new Map([[player, null]]);
    const q = [player];
    while (q.length) {
      const c = q.shift();
      if (c === target) break;
      for (let d = 0; d < 4; d++) {
        const nx = nbr[c * 4 + d];
        if (nx < 0 || wall[nx] || boxes.has(nx) || prev.has(nx)) continue;
        prev.set(nx, [c, d]);
        q.push(nx);
      }
    }
    if (!prev.has(target)) return false;
    const letters = [];
    for (let c = target; prev.get(c); c = prev.get(c)[0]) letters.push(LET[prev.get(c)[1]]);
    letters.reverse();
    out += letters.join('');
    player = target;
    return true;
  };
  for (const p of pushes) {
    const stand = nbr[p.box * 4 + (p.dir ^ 1)];
    if (!walkTo(stand)) return null;
    const dest = nbr[p.box * 4 + p.dir];
    boxes.delete(p.box);
    boxes.add(dest);
    out += LET[p.dir];
    player = p.box;
  }
  return out;
}

function solveGlobal(L, T, budget) {
  const n = L.w * L.h;
  const all = new Uint8Array(n).fill(1);
  for (let i = 0; i < n; i++) if (L.wall[i]) all[i] = 0;
  const boxes = L.boxes.slice().sort((a, b) => a - b);
  const res = solvePush(L, T, all, all, boxes, L.goals, L.player, budget);
  if (!res) return null;
  const moves = expandMoves(L, boxes, L.player, res.pushes);
  return moves === null ? null : { moves, pushes: res.pushes.length, nodes: res.nodes };
}

function findRooms(L) {
  const { wall, nbr } = L;
  const n = L.w * L.h;
  const boxAt = new Set(L.boxes);
  const goalAt = new Set(L.goals);
  const work = new Set();
  for (let i = 0; i < n; i++) if (!wall[i]) work.add(i);
  const claimedDoor = new Set();
  const rooms = [];
  const components = (removed) => {
    const seenSet = new Set();
    const comps = [];
    for (const s of work) {
      if (s === removed || seenSet.has(s)) continue;
      const comp = new Set([s]);
      seenSet.add(s);
      const stack = [s];
      while (stack.length) {
        const c = stack.pop();
        for (let d = 0; d < 4; d++) {
          const q = nbr[c * 4 + d];
          if (q < 0 || q === removed || !work.has(q) || seenSet.has(q)) continue;
          seenSet.add(q);
          comp.add(q);
          stack.push(q);
        }
      }
      comps.push(comp);
    }
    return comps;
  };
  for (;;) {
    let bestCand = null;
    for (const c of work) {
      if (c === L.player || claimedDoor.has(c)) continue;
      for (const X of components(c)) {
        if (X.has(L.player)) continue;
        let bx = boxAt.has(c) ? 1 : 0;
        let gl = goalAt.has(c) ? 1 : 0;
        for (const cell of X) {
          if (rooms.some((r) => r.cells.has(cell))) continue;
          if (boxAt.has(cell)) bx++;
          if (goalAt.has(cell)) gl++;
        }
        if (bx >= 1 && bx === gl && (!bestCand || X.size > bestCand.cells.size)) {
          bestCand = { door: c, cells: new Set([...X, c]) };
        }
      }
    }
    if (!bestCand) break;
    rooms.push(bestCand);
    claimedDoor.add(bestCand.door);
    for (const cell of bestCand.cells) if (cell !== bestCand.door) work.delete(cell);
  }
  const covered = new Set();
  for (const r of rooms) for (const cell of r.cells) covered.add(cell);
  for (const b of L.boxes) if (!covered.has(b)) return null;
  for (const g of L.goals) if (!covered.has(g)) return null;
  return rooms;
}

function solveRooms(L, T, perRoomBudget) {
  const rooms = findRooms(L);
  if (!rooms) return null;
  const n = L.w * L.h;
  const allPushes = [];
  const curBoxes = L.boxes.slice().sort((a, b) => a - b);
  let totalNodes = 0;
  for (const room of rooms) {
    let entry = -1;
    for (let d = 0; d < 4; d++) {
      const q = L.nbr[room.door * 4 + d];
      if (q < 0 || L.wall[q]) continue;
      if (!room.cells.has(q)) { entry = q; break; }
    }
    if (entry < 0) return null;
    const boxCells = new Uint8Array(n);
    const walkCells = new Uint8Array(n);
    for (const cell of room.cells) { boxCells[cell] = 1; walkCells[cell] = 1; }
    walkCells[entry] = 1;
    const rBoxes = curBoxes.filter((b) => room.cells.has(b)).sort((a, b) => a - b);
    const rGoals = L.goals.filter((g) => room.cells.has(g)).sort((a, b) => a - b);
    if (rBoxes.length !== rGoals.length || rBoxes.length === 0) return null;
    const res = solvePush(L, T, boxCells, walkCells, rBoxes, rGoals, entry, perRoomBudget);
    if (!res) return null;
    totalNodes += res.nodes;
    const sim = new Set(curBoxes);
    for (const p of res.pushes) {
      sim.delete(p.box);
      sim.add(L.nbr[p.box * 4 + p.dir]);
      allPushes.push(p);
    }
    curBoxes.length = 0;
    curBoxes.push(...[...sim].sort((a, b) => a - b));
  }
  const startBoxes = L.boxes.slice().sort((a, b) => a - b);
  const moves = expandMoves(L, startBoxes, L.player, allPushes);
  if (moves === null) return null;
  const endBoxes = new Set(L.boxes);
  let player = L.player;
  for (const ch of moves) {
    const d = LET.indexOf(ch);
    const nx = L.nbr[player * 4 + d];
    if (endBoxes.has(nx)) {
      endBoxes.delete(nx);
      endBoxes.add(L.nbr[nx * 4 + d]);
    }
    player = nx;
  }
  for (const b of endBoxes) if (!L.goals.includes(b)) return null;
  return { moves, pushes: allPushes.length, nodes: totalNodes };
}

function solveLevel(text) {
  const L = parseLevel(text);
  const T = pushTables(L);
  const r = solveRooms(L, T, 800_000);
  if (r) return r;
  const g = solveGlobal(L, T, 400_000);
  if (g) return g;
  return { moves: '', pushes: 0, nodes: 0 };
}

for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8')).moves;
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
