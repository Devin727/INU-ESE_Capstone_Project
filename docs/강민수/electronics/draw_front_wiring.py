"""Render a module-level wiring proposal, not a PCB/manufacturing schematic.
Run: python draw_front_wiring.py  (Pillow; Windows Malgun Gothic font).
SVG, PNG and PDF share the same source geometry. No hardware commands.
"""
from pathlib import Path
from html import escape
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
W, H = 2400, 1900
FONT = Path('C:/Windows/Fonts/malgun.ttf')
BOLD = Path('C:/Windows/Fonts/malgunbd.ttf')
im = Image.new('RGB', (W, H), '#f5f7fb')
d = ImageDraw.Draw(im)
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
       '<rect width="100%" height="100%" fill="#f5f7fb"/>']
INK, BLUE, RED, GREEN, AMBER = '#17263b', '#2463b7', '#c33e48', '#16806d', '#9c650d'


def text(x, y, value, size=25, color=INK, bold=False):
    font = ImageFont.truetype(str(BOLD if bold else FONT), size)
    d.text((x, y), value, font=font, fill=color)
    svg.append(f'<text x="{x}" y="{y}" dominant-baseline="text-before-edge" '
               f'font-family="Malgun Gothic, sans-serif" font-size="{size}" '
               f'font-weight="{700 if bold else 400}" fill="{color}">{escape(value)}</text>')


def line(points, color=INK, width=3, dashed=False):
    if dashed:
        # All dashed connections in this drawing are orthogonal.
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            length = abs(x1-x0) + abs(y1-y0)
            for n in range(0, int(length), 18):
                a, b = n/length, min(n+9, length)/length
                d.line([(x0+(x1-x0)*a, y0+(y1-y0)*a),
                        (x0+(x1-x0)*b, y0+(y1-y0)*b)], fill=color, width=width)
    else:
        d.line(points, fill=color, width=width)
    coords = ' '.join(f'{x},{y}' for x, y in points)
    dash = ' stroke-dasharray="9 9"' if dashed else ''
    svg.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"{dash}/>')


def box(x, y, w, h, title, color=INK, fill='white'):
    d.rounded_rectangle((x, y, x+w, y+h), radius=14, fill=fill, outline=color, width=3)
    svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="14" fill="{fill}" stroke="{color}" stroke-width="3"/>')
    text(x+22, y+18, title, 29, color, True)
    line([(x+18, y+65), (x+w-18, y+65)], '#dce3ed', 2)


text(65, 35, '앞 모듈 모터 구동 연결도  v0.2', 48, bold=True)
text(65, 101, 'FIT0522 ×3 (앞 모듈)  ·  MDD3A ×2 · ESP32 DevKitC / WROOM-32D (뒤 마디, 기구 v2)  ·  모터선 6가닥 ×3이 관절 통과', 28)
text(65, 145, '2026-11-23 (연결 내용은 2026-11-19 초안과 같음)  |  모듈 단위 배선 초안 · GPIO 제안 · 실물 미검증 · 커넥터 물리 순서 아님', 24, AMBER)

box(65, 205, 2270, 185, 'J_PWR  외부 / 뒤마디에서 공급 — 전원부 모델·퓨즈·전선 굵기 미정', RED)
text(90, 286, '+6V_MOTOR → U2·U3 VB+     |     +5V_LOGIC → U1 5V     |     GND → 전체 공통', 29, RED, True)
text(90, 338, '모터 전원 예산 [계산]: 최대출력점 합 3 A / 동시 스톨 합 9 A.  2S 배터리 원전압을 모터에 직접 연결하지 않음.', 24)

# Signal labels inside modules are electrical net names, not physical connector order.
box(65, 430, 680, 930, 'U1  ESP32 DevKitC / WROOM-32D', BLUE)
text(90, 510, '5V ← +5V_LOGIC    GND ↔ 공통 GND', 25, RED)
text(90, 553, '3V3 → 엔코더 VCC ×3 (전류 합 확인 필요)', 24, RED)
text(90, 610, '모터 제어 출력 [GPIO 제안]', 25, BLUE, True)
controls = [('GPIO25', 'DRV1_A', 'U2 IN M1A'), ('GPIO26', 'DRV1_B', 'U2 IN M1B'),
            ('GPIO27', 'DRV2_A', 'U2 IN M2A'), ('GPIO14', 'DRV2_B', 'U2 IN M2B'),
            ('GPIO32', 'DRV3_A', 'U3 IN M1A'), ('GPIO33', 'DRV3_B', 'U3 IN M1B')]
for i, (gpio, net, target) in enumerate(controls):
    text(93, 655+i*39, f'{gpio} → {net} → {target}', 24)
text(90, 918, '엔코더 입력 [GPIO 제안]', 25, GREEN, True)
enc = [('ENC1_A / B', 'GPIO34 / 35'), ('ENC2_A / B', 'GPIO36 / 39'), ('ENC3_A / B', 'GPIO18 / 19')]
for i, (net, gpio) in enumerate(enc):
    text(93, 963+i*40, f'{net} → {gpio}', 25, GREEN)
text(90, 1110, 'ToF 예비: SDA21 / SCL22', 25, AMBER)
text(90, 1150, '선택 제어선 예비: GPIO13 / GPIO23', 24, AMBER)
text(90, 1210, '상위 통신 예비 [가정]: TX17 / RX16', 24)
text(90, 1250, '3.3 V UART · 상대 RX/TX 교차 · GND 공통', 23)
text(90, 1300, 'USB 전원과 외부 5V는 동시 공급 금지', 24, RED, True)

box(1010, 430, 610, 415, 'U2  MDD3A #1 — 모터 1·2', BLUE)
text(1035, 512, 'VB+ ← +6V_MOTOR   VB- / GND → GND', 24, RED)
for i, (_, net, target) in enumerate(controls[:4]):
    text(1035, 560+i*38, f'IN {target.split("IN ")[1]} ← {net}', 24, BLUE)
text(1035, 728, 'OUT CH1 A/B → M1 핀 6 / 1', 25)
text(1035, 767, 'OUT CH2 A/B → M2 핀 6 / 1', 25)
text(1035, 805, '5VO: 미연결 (200 mA 출력)', 23, AMBER)

box(1010, 895, 610, 340, 'U3  MDD3A #2 — 모터 3', BLUE)
text(1035, 975, 'VB+ ← +6V_MOTOR   VB- / GND → GND', 24, RED)
text(1035, 1020, 'IN M1A / M1B ← DRV3_A / DRV3_B', 24, BLUE)
text(1035, 1065, 'OUT CH1 A/B → M3 핀 6 / 1', 25)
text(1035, 1110, '미사용 IN M2A / M2B → 각각 10 kΩ → GND', 22)
text(1035, 1152, '미사용 OUT CH2: 개방 · 다른 출력과 병렬 금지', 22)
text(1035, 1193, '5VO: 미연결', 23, AMBER)

for i, y in enumerate((430, 710, 990), 1):
    box(1850, y, 485, 240, f'M{i}  FIT0522 + 엔코더', GREEN)
    text(1870, y+81, f'핀6 / 핀1 ← U{2 if i < 3 else 3} OUT CH{2 if i == 2 else 1} A/B', 23)
    text(1870, y+123, '핀5 VCC ← 3V3   핀2 GND → GND', 23, RED)
    text(1870, y+165, f'핀3 / 핀4 → ENC{i}_A / ENC{i}_B', 23, GREEN)
    text(1870, y+204, '핀1은 모터선! 공통 GND에 직결하지 않음', 21, RED)

line([(1620, 748), (1730, 748), (1730, 550), (1850, 550)], BLUE, 4)
line([(1620, 787), (1690, 787), (1690, 830), (1850, 830)], BLUE, 4)
line([(1620, 1084), (1730, 1084), (1730, 1110), (1850, 1110)], BLUE, 4)
text(1660, 465, '모터선 2가닥씩', 21, BLUE)
line([(745, 735), (870, 735), (870, 620), (1010, 620)], BLUE, 4)
text(770, 655, 'DRV1/2', 23, BLUE)
line([(745, 850), (825, 850), (825, 1035), (1010, 1035)], BLUE, 4)
text(840, 992, 'DRV3', 23, BLUE)

box(1010, 1280, 1325, 245, 'ToF 예비 연결 — 부품 미선정 / 현재 장착하지 않음', AMBER, '#fffaf0')
text(1035, 1360, 'I²C형을 택할 경우 [가정]: SDA ↔ GPIO21 / SCL ↔ GPIO22 / GND 공통', 25)
text(1035, 1407, 'V_TOF는 미연결 유지. 구매 보드의 전원·논리 전압·단자 순서 확인 후 결정.', 24, AMBER)
text(1035, 1454, 'XSHUT / INT는 선택 예약. 풀업은 보드 내장 여부 확인 후 3.3 V 기준으로 구성.', 24)
line([(745, 1130), (905, 1130), (905, 1405), (1010, 1405)], AMBER, 3, True)

box(65, 1410, 680, 390, '정지 동작 / 기존 코드와 차이', RED, '#fff7f7')
for i, row in enumerate(['MDD3A: A=0, B=0 → 브레이크', 'A=PWM, B=0 / A=0, B=PWM → 정·역',
                         'A=1, B=1 → 브레이크', '기존 DRV8833 코드의 00=Coast와 다름.',
                         'MDD3A에는 nSLEEP 단자가 없음.', '핀 설정·정지 의미 수정 전 실물 구동 금지.']):
    text(90, 1495+i*43, row, 24)

box(790, 1570, 1545, 230, '추가 수동 부품 [제안값 / 미구매·미검증]', INK)
text(815, 1650, '각 제어 입력 → 10 kΩ → GND (활성 6개 + 미사용 2개, 합 8개)', 25)
text(815, 1694, '각 드라이버 VB+ ↔ GND: 470 µF / 16 V + 100 nF 병렬. 각 모터 핀6 ↔ 핀1: 100 nF.', 24)
text(815, 1738, '엔코더 A/B 풀업: 출력 회로 확인 전 DNP. 필요 시 4.7 kΩ → 3V3 (6개).', 24, GREEN)
text(65, 1835, '공통 GND: 모터 전류 귀환은 전원 분배점으로 직접. ESP32·엔코더 접지 배선을 모터 전류 경로로 쓰지 않음.', 25)
svg.append('</svg>')
(OUT/'front_module_wiring.svg').write_text('\n'.join(svg), encoding='utf-8')
im.save(OUT/'front_module_wiring.png')
im.save(OUT/'front_module_wiring.pdf', 'PDF', resolution=180)
print('Wrote front_module_wiring.svg / .png / .pdf')
