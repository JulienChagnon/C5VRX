// C5VRX-5 by Twotoz and the C5VRX contributors: wiring pictures for the
// flasher, drawn from web/boards.js and the pins in use (the board's own or
// the pin editor's). Plain SVG strings; colours come from style.css (.bd-*).
import { DAC_OHMS, DAC_LOAD_OHMS, IQ_NAMES, ohms } from './boards.js';

const esc = text => String(text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
const POWER = /^(5V|3V3|VBUS|VSYS|VIN)$/i;
const GROUND = /^(GND|G)$/i;

// What each GPIO does with these pins: { kind, text } by GPIO number.
export function pinRoles(pins) {
  const roles = new Map();
  pins.dac.forEach((p, k) => roles.set(p, { kind: 'dac', text: `D${4 + k} · ${ohms(DAC_OHMS[k])}`, bit: k }));
  pins.iq.forEach((p, k) => roles.set(p, { kind: 'iq', text: `leave free (IQ ${IQ_NAMES[k]})` }));
  roles.set(pins.boot, { kind: 'boot', text: 'menu button' });
  return roles;
}

function chip(x, y, width, role, anchorRight) {
  const left = anchorRight ? x - width : x;
  const cls = `bd-chip bd-chip-${role.kind}`;
  return `<rect class="${cls}" x="${left}" y="${y - 9}" width="${width}" height="18" rx="4"/>` +
    `<text class="bd-chip-text bd-chip-text-${role.kind}" x="${left + width / 2}" y="${y + 4}" text-anchor="middle">${esc(role.text)}</text>`;
}

// The board seen from the top, both headers in their real order.
export function boardSvg(board, layout, pins, bad = new Set()) {
  const roles = pinRoles(pins);
  const pitch = 24, top = 70;
  const rows = Math.max(layout.left.length, layout.right.length);
  const boardTop = top - 26, boardH = rows * pitch + 36;
  const height = boardTop + boardH + 90;
  const bx = 300, bw = 160, lx = bx + 18, rx = bx + bw - 18;
  let groundNoted = false;
  const out = [`<svg class="bd-svg" viewBox="0 0 760 ${height}" role="img" aria-label="${esc(`${board.name} ${layout.name}: where each part goes`)}" xmlns="http://www.w3.org/2000/svg">`];
  out.push(`<text class="bd-title" x="380" y="18" text-anchor="middle">${esc(board.name)} · ${esc(layout.name)}</text>`);
  out.push(`<rect class="bd-board" x="${bx}" y="${boardTop}" width="${bw}" height="${boardH}" rx="10"/>`);
  if (/usb/i.test(layout.top)) out.push(`<rect class="bd-conn" x="${bx + bw / 2 - 26}" y="${boardTop - 12}" width="52" height="22" rx="4"/><text class="bd-small" x="${bx + bw / 2}" y="${boardTop + 3}" text-anchor="middle">USB-C</text>`);
  else if (layout.top) out.push(`<path class="bd-antenna" d="M${bx + 40} ${boardTop + 16} h12 v-8 h12 v8 h12 v-8 h12 v8 h12 v-8 h12 v8 h12"/><text class="bd-small" x="${bx + bw / 2}" y="${boardTop + 30}" text-anchor="middle">${esc(layout.top)}</text>`);
  if (layout.bottom) out.push(`<text class="bd-small" x="${bx + bw / 2}" y="${boardTop + boardH - 10}" text-anchor="middle">${esc(layout.bottom)}</text>`);
  const side = (pinsOnSide, px, right) => {
    pinsOnSide.forEach(([gpio, silk], i) => {
      const y = top + i * pitch;
      const role = gpio === null ? null : roles.get(gpio);
      const part = gpio === null ? null : board.parts[gpio];
      const kind = role ? role.kind : gpio === null ? (GROUND.test(silk) ? 'gnd' : POWER.test(silk) ? 'pwr' : 'none') : 'free';
      out.push(`<circle class="bd-pin bd-pin-${kind}${bad.has(gpio) ? ' bd-bad' : ''}" cx="${px}" cy="${y}" r="6"/>`);
      out.push(`<text class="bd-silk" x="${right ? px - 11 : px + 11}" y="${y + 4}" text-anchor="${right ? 'end' : 'start'}">${esc(silk)}</text>`);
      const edge = right ? bx + bw : bx;
      const textX = right ? edge + 10 : edge - 10;
      const anchor = right ? 'start' : 'end';
      if (gpio !== null) out.push(`<text class="bd-gpio" x="${textX}" y="${y + 4}" text-anchor="${anchor}">GPIO${gpio}</text>`);
      const chipX = right ? edge + 62 : edge - 62;
      // One GND carries the DAC load and the AV ground; the others stay plain.
      let label = role;
      if (!label && kind === 'gnd' && !groundNoted) { label = { kind: 'gnd', text: `GND: ${ohms(DAC_LOAD_OHMS)} load, AV ground` }; groundNoted = true; }
      else if (!label && kind === 'pwr') label = { kind: 'pwr', text: /3V3/i.test(silk) ? '3.3 V' : /VSYS/i.test(silk) ? 'VSYS (battery or USB)' : '5 V' };
      if (label) {
        out.push(`<line class="bd-lead bd-lead-${label.kind}" x1="${px + (right ? 6 : -6)}" y1="${y}" x2="${right ? edge + 4 : edge - 4}" y2="${y}"/>`);
        out.push(chip(chipX, y, label.kind === 'gnd' && label.text !== 'GND' ? 178 : 132, label, !right));
      } else if (part) {
        // An onboard part on an unused pin; a part under a job is a pin editor warning.
        out.push(`<text class="bd-part" x="${chipX}" y="${y + 4}" text-anchor="${anchor}">${esc(part)}</text>`);
      }
    });
  };
  side(layout.left, lx, false);
  side(layout.right, rx, true);
  // Pins in use that no header carries (pads inside the board).
  const onHeader = new Set([...layout.left, ...layout.right].map(([g]) => g));
  const inside = [...roles.entries()].filter(([g]) => !onHeader.has(g));
  // IQ loopbacks and the onboard BOOT button need no wire; a DAC branch does.
  const internal = inside.filter(([g, r]) => r.kind === 'iq' || (r.kind === 'boot' && board.parts[g]));
  const unwirable = inside.filter(entry => !internal.includes(entry));
  let y = boardTop + boardH + 22;
  if (internal.length) {
    const text = internal.map(([g, r]) => (r.kind === 'iq' ? `GPIO${g}` : `GPIO${g} (BOOT button)`)).join(', ');
    out.push(`<text class="bd-small" x="380" y="${y}" text-anchor="middle">Inside the board, nothing to wire: ${esc(text)}</text>`);
    y += 20;
  }
  if (unwirable.length) {
    const text = unwirable.map(([g, r]) => `${r.kind === 'boot' ? 'menu button' : r.text.split(' · ')[0]} on GPIO${g}`).join(', ');
    out.push(`<text class="bd-small bd-unwirable" x="380" y="${y}" text-anchor="middle">Not on a header, cannot be wired: ${esc(text)}</text>`);
    y += 20;
  }
  out.push(`<text class="bd-small" x="380" y="${y}" text-anchor="middle">Blue: a DAC resistor from this pin to the video node. Dashed: the receiver drives it, leave it unconnected.</text>`);
  out.push('</svg>');
  return out.join('');
}

// The resistor DAC as a schematic, with the GPIO of every branch.
export function ladderSvg(pins) {
  const out = ['<svg class="bd-svg" viewBox="0 0 760 272" role="img" aria-label="Resistor DAC wiring" xmlns="http://www.w3.org/2000/svg">'];
  out.push('<text class="bd-title" x="380" y="18" text-anchor="middle">Resistor DAC: one resistor per pin into the video node</text>');
  const bus = 470;
  pins.dac.forEach((p, k) => {
    const y = 44 + k * 28;
    out.push(`<text class="bd-gpio" x="118" y="${y + 4}" text-anchor="end">GPIO${p}</text>`);
    out.push(`<text class="bd-silk" x="130" y="${y + 4}">D${4 + k}</text>`);
    out.push(`<line class="bd-wire" x1="160" y1="${y}" x2="250" y2="${y}"/>`);
    out.push(`<rect class="bd-res" x="250" y="${y - 8}" width="90" height="16" rx="2"/>`);
    out.push(`<text class="bd-res-text" x="295" y="${y + 4}" text-anchor="middle">${esc(ohms(DAC_OHMS[k]))}</text>`);
    out.push(`<line class="bd-wire" x1="340" y1="${y}" x2="${bus}" y2="${y}"/>`);
    out.push(`<circle class="bd-node" cx="${bus}" cy="${y}" r="3"/>`);
  });
  const lastY = 44 + 5 * 28;
  out.push(`<line class="bd-wire" x1="${bus}" y1="44" x2="${bus}" y2="${lastY + 18}"/>`);
  out.push(`<text class="bd-small" x="${bus + 8}" y="40">video node</text>`);
  // 200 ohm load to GND.
  out.push(`<rect class="bd-res" x="${bus - 8}" y="${lastY + 18}" width="16" height="34" rx="2"/>`);
  out.push(`<text class="bd-res-text" x="${bus - 14}" y="${lastY + 39}" text-anchor="end">${esc(ohms(DAC_LOAD_OHMS))}</text>`);
  out.push(`<line class="bd-wire" x1="${bus}" y1="${lastY + 52}" x2="${bus}" y2="${lastY + 60}"/>`);
  out.push(`<path class="bd-wire" d="M${bus - 12} ${lastY + 60} h24 M${bus - 7} ${lastY + 64} h14 M${bus - 3} ${lastY + 68} h6"/>`);
  out.push(`<text class="bd-small" x="${bus}" y="${lastY + 84}" text-anchor="middle">GND</text>`);
  // To the goggles or monitor.
  const jy = 114;
  out.push(`<line class="bd-wire" x1="${bus}" y1="${jy}" x2="640" y2="${jy}"/>`);
  out.push(`<circle class="bd-jack" cx="660" cy="${jy}" r="16"/><circle class="bd-node" cx="660" cy="${jy}" r="4"/>`);
  out.push(`<text class="bd-small" x="660" y="${jy - 24}" text-anchor="middle">AV video in</text>`);
  out.push(`<text class="bd-small" x="660" y="${jy + 34}" text-anchor="middle">centre: video</text>`);
  out.push(`<text class="bd-small" x="660" y="${jy + 48}" text-anchor="middle">shield: GND</text>`);
  out.push(`<path class="bd-wire bd-wire-gnd" d="M676 ${jy} h26 v${lastY + 60 - jy} H${bus + 12}"/>`);
  out.push('</svg>');
  return out.join('');
}

// Boards without a known layout: every GPIO and its job.
export function pinGridSvg(pins, bad = new Set()) {
  const roles = pinRoles(pins);
  const reserved = new Map([[13, 'USB D-'], [14, 'USB D+'], ...[16, 17, 18, 19, 20, 21, 22].map(g => [g, 'flash'])]);
  const out = ['<svg class="bd-svg" viewBox="0 0 760 252" role="img" aria-label="ESP32-C5 GPIO jobs" xmlns="http://www.w3.org/2000/svg">'];
  out.push('<text class="bd-title" x="380" y="18" text-anchor="middle">ESP32-C5 GPIOs with these pins</text>');
  for (let g = 0; g <= 28; ++g) {
    const col = g % 8, row = Math.floor(g / 8);
    const x = 24 + col * 90, y = 34 + row * 48;
    const role = roles.get(g);
    const kind = role ? role.kind : reserved.has(g) ? 'reserved' : 'free';
    const text = role ? (role.kind === 'iq' ? `IQ ${IQ_NAMES[pins.iq.indexOf(g)]}` : role.kind === 'boot' ? 'button' : `D${4 + role.bit}`) : reserved.get(g) || '';
    out.push(`<rect class="bd-cell bd-cell-${kind}${bad.has(g) ? ' bd-bad' : ''}" x="${x}" y="${y}" width="82" height="40" rx="5"/>`);
    out.push(`<text class="bd-gpio" x="${x + 41}" y="${y + 16}" text-anchor="middle">GPIO${g}</text>`);
    if (text) out.push(`<text class="bd-cell-text bd-cell-text-${kind}" x="${x + 41}" y="${y + 32}" text-anchor="middle">${esc(text)}</text>`);
  }
  out.push('<text class="bd-small" x="380" y="244" text-anchor="middle">Blue: DAC resistor. Dashed: IQ loopback, leave unconnected. Grey: USB and flash, never usable.</text>');
  out.push('</svg>');
  return out.join('');
}

// GPIOs that take part in a rule the pins break, for highlighting.
export function badPins(pins) {
  const all = [...pins.dac, ...pins.iq, pins.boot];
  const bad = new Set();
  all.forEach((p, k) => {
    if (all.indexOf(p) !== k) bad.add(p);
    if (p === 13 || p === 14 || (p >= 16 && p <= 22) || p < 0 || p > 28) bad.add(p);
    if (p === 28 && k < 14) bad.add(p);
  });
  return bad;
}
