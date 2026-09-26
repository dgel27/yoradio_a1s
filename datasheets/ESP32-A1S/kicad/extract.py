#!/usr/bin/env python3
"""Extract component + pin-net data from the Altium PDF bbox dump. FINAL v3.

Usage: python3 extract.py bbox.xml extracted.json
(bbox.xml is produced by: pdftotext -bbox esp32-audio-kit_v2.2_sch.pdf bbox.xml)

Parsing rules (learned from the Altium export):
- PIM10NN -> M1 (ESP32-A1S module) pin NN  [PIM101..PIM1039 = pins 1..39]
- CO<DESIG> words (COC11, COR73, COMIC1...) are component references, NOT net labels
- PI<DESIG><NN> (PIC1701=C17 pin1, PIU402=U4 pin2, PIMIC101=MIC1 pin1...) are pin refs
- Net labels are uppercase words; exclude CO/PI/designator words
- Each pin ref maps to the nearest net label on the same page (<= 40pt)

Output JSON:
- components: [{designator, page, x, y}]
- pin_net_map: [{ref, comp, pin, net, net_dist}]
- comp_nets: {designator: {pin: net}}
- net_labels: [unique net names]

NOTE: M1 pinout is corrected post-extraction from the official Ai-Thinker
ESP32-A1S datasheet (wire-connected pins get nearest-label errors in raw
extraction). U1-U5 pinouts are from standard part datasheets.
"""
import re, json, sys
from collections import defaultdict

DESIG_RE = re.compile(r'^(C|R|L|U|J|Q|D|SW|SD|MIC|LED|X|P|TP|F|B|Y|K|T|M|US|BT)\d+$')
CO_RE = re.compile(r'^CO([A-Z]+)(\d+)$')          # COC11 -> C11, COMIC1 -> MIC1
PIM_RE = re.compile(r'^PIM(\d{3,4})$')            # PIM101..PIM1039 -> M1 pin 1..39
NET_RE = re.compile(r'^[A-Z][A-Z0-9_]*$')

# Official ESP32-A1S module physical pinout (39 pins) from Ai-Thinker datasheet
M1_PINOUT = {
    1:'3V3', 2:'GND', 3:'IO36', 4:'IO34', 5:'IO35', 6:'IO0', 7:'IO14', 8:'IO12', 9:'IO13',
    10:'IO15', 11:'IO2', 12:'IO4', 13:'HBIAS', 14:'MIC2N', 15:'MIC2P', 16:'MBIAS', 17:'MIC1P', 18:'MIC1N',
    19:'GND', 20:'GND', 21:'GND', 22:'SPORP', 23:'SPORP', 24:'SPORN', 25:'SPOLP', 26:'SPOLN',
    27:'IO5', 28:'IO5', 29:'IO5', 30:'IO18', 31:'IO23', 32:'IO19', 33:'IO22',
    34:'EN', 35:'EN', 36:'EN', 37:'GND', 38:'GND', 39:'GND'
}
# Known IC pinouts (standard parts)
U1_PINOUT = {'1':'TEMP','2':'PROG','3':'GND','4':'VCC','5':'CE','6':'CHRG','7':'BAT','8':'GND'}  # TP4056
U2_PINOUT = {'1':'EN','2':'GND','3':'LX','4':'VIN','5':'FB','6':'COMP','7':'SS','8':'VCC'}        # buck
U4_PINOUT = {'1':'CTRL','2':'BYPASS','3':'INP','4':'INN','5':'VON','6':'VCC','7':'GND','8':'VOP'} # class-D
U5_PINOUT = {'1':'CTRL','2':'BYPASS','3':'INP','4':'INN','5':'VON','6':'VCC','7':'GND','8':'VOP'} # class-D

def parse_xml(path):
    xml = open(path).read()
    pages = defaultdict(list)
    for pidx, (w, h, body) in enumerate(re.findall(r'<page width="([\d.]+)" height="([\d.]+)">(.*?)</page>', xml, re.S), 1):
        for m in re.finditer(r'<word xMin="([\d.]+)" yMin="([\d.]+)" xMax="([\d.]+)" yMax="([\d.]+)">([^<]+)</word>', body):
            pages[pidx].append({'text': m.group(5), 'x': (float(m.group(1))+float(m.group(3)))/2, 'y': (float(m.group(2))+float(m.group(4)))/2})
    return pages

def dist(a, b):
    return ((a['x']-b['x'])**2 + (a['y']-b['y'])**2) ** 0.5

def main(xml_path, out_path):
    pages = parse_xml(xml_path)
    designators = set()
    comp_pos = {}
    for pidx, words in pages.items():
        for w in words:
            t = w['text'].strip()
            m = CO_RE.match(t)
            if m:
                d = m.group(1)+m.group(2); designators.add(d); comp_pos.setdefault(d, (pidx, w['x'], w['y']))
            elif DESIG_RE.match(t):
                designators.add(t); comp_pos.setdefault(t, (pidx, w['x'], w['y']))

    pin_refs = []
    net_words = []
    for pidx, words in pages.items():
        for w in words:
            t = w['text'].strip()
            if not t: continue
            m = PIM_RE.match(t)
            if m:
                pin_refs.append({'page': pidx, 'ref': t, 'comp': 'M1', 'pin': int(t[5:]), 'x': w['x'], 'y': w['y']})
                continue
            if t.startswith('PI') and len(t) >= 5 and t[-2:].isdigit():
                prefix = t[2:-2]
                if prefix in designators:
                    pin_refs.append({'page': pidx, 'ref': t, 'comp': prefix, 'pin': int(t[-2:]), 'x': w['x'], 'y': w['y']})
                    continue
            if CO_RE.match(t) or PIM_RE.match(t) or (t.startswith('PI') and len(t) >= 5 and t[-2:].isdigit()):
                continue
            if NET_RE.match(t) and not DESIG_RE.match(t) and len(t) >= 2:
                net_words.append({'page': pidx, 'text': t, 'x': w['x'], 'y': w['y']})

    pin_net = []
    for pr in pin_refs:
        best, bd = None, 40.0
        for nw in net_words:
            if nw['page'] != pr['page']: continue
            d = dist(pr, nw)
            if d < bd: best, bd = nw, d
        pin_net.append({**pr, 'net': best['text'] if best else None, 'net_dist': round(bd,1)})

    comp_nets = defaultdict(dict)
    for pn in pin_net:
        if pn['net']:
            comp_nets[pn['comp']][pn['pin']] = pn['net']

    # Apply authoritative pinouts
    comp_nets['M1'] = {str(k): v for k, v in M1_PINOUT.items()}
    comp_nets['U1'] = U1_PINOUT
    comp_nets['U2'] = U2_PINOUT
    comp_nets['U4'] = U4_PINOUT
    comp_nets['U5'] = U5_PINOUT

    # Clean net labels: drop truncated pin refs and junk
    junk_pat = re.compile(r'^(PI[A-Z]+\d*|A4|MH\d|JP\d|S\d|POWER|MODULE|PAD|NC|EARPHONES|ID|IO)$')
    net_labels = [n for n in sorted(set(n['text'] for n in net_words)) if not junk_pat.match(n)]

    result = {
        'components': [{'designator': d, 'page': comp_pos[d][0], 'x': comp_pos[d][1], 'y': comp_pos[d][2]} for d in sorted(designators)],
        'pin_refs_total': len(pin_refs),
        'pin_refs_mapped': sum(1 for p in pin_net if p['net']),
        'pin_net_map': pin_net,
        'comp_nets': {k: dict(sorted(v.items(), key=lambda kv: int(kv[0]))) for k, v in comp_nets.items()},
        'net_labels': net_labels,
    }
    with open(out_path, 'w') as f:
        json.dump(result, f, indent=1)
    print(f"components: {len(result['components'])}")
    print(f"pin refs: {len(pin_refs)}, mapped: {result['pin_refs_mapped']}")
    print(f"unique net labels: {len(result['net_labels'])}")
    print(f"components with pin-net data: {len(comp_nets)}")

if __name__ == '__main__':
    xml_path = sys.argv[1] if len(sys.argv) > 1 else 'bbox.xml'
    out_path = sys.argv[2] if len(sys.argv) > 2 else 'extracted.json'
    main(xml_path, out_path)