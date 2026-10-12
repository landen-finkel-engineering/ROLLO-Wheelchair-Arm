# 16 — Shopping list (where to buy each purchased part)

This is the **Purchased parts** section of [`output/reports/BOM.md`](../output/reports/BOM.md) (ABENICS wrist, WRA-1A) with a link for each item, mostly Amazon.

- **Links were picked on 2026-10-12 from the listing titles.** Prices, pack sizes and stock change. Before you buy, check the listing against the "need" column; where a listing has options, the note says which to choose.
- **Two items are on hold** until the design is finished: the gravity-balance springs and the 30 mm J1 shaft collar. See [On hold](#on-hold) at the end.
- **Screws, nuts and washers are not in this list.** They are in the *Fastener schedule* section of `BOM.md`.
- **Cut-to-length parts.** Several parts are sold longer than needed. The hardened steel shafts and the MGN9 rail need an abrasive cut-off wheel (angle grinder or rotary tool), not a hacksaw.

## Servos, controller and modules

| item | need | buy | notes |
|---|---|---|---|
| Waveshare ST3215 serial bus servo, 30 kg·cm @ 12 V (SKU 22414) | 9 (8 + 1 spare) | [Amazon](https://www.amazon.com/dp/B0CFY52HVV) | Must say **30 kg·cm @ 12 V**. A 7.4 V / 19.5 kg·cm version also exists and won't work |
| Waveshare Bus Servo Adapter (A) | 1 | [Amazon](https://www.amazon.com/dp/B0CJBKQ8LJ) | the 42 × 33 mm board with UART and USB |
| 5264 3-pin servo leads | for the daisy chain | [Waveshare 6-pack, 3 × 300 mm + 3 × 900 mm](https://www.amazon.com/dp/B0GVDFXF7Q) | part of the BOM's wiring line |
| ESP32-WROOM-32 dev board | 1 | you already have one (Freenove); otherwise [search](https://www.amazon.com/s?k=ESP32-WROOM-32+DevKitC) | |
| Jetson Orin Nano 8GB, ReSpeaker USB mic | — | you already have these | |

## Power

| item | need | buy | notes |
|---|---|---|---|
| DC-DC buck 12 V / 20 A for the servos | 1 | [Amazon](https://www.amazon.com/dp/B08YYJMGJ1) | input 15–40 V, so it still works when the battery is nearly empty |
| DC-DC converter for the Jetson | 1 | [12 V / 10 A, input 15–40 V](https://www.amazon.com/dp/B0922KFBDH) + [5.5 × 2.5 mm barrel-plug pigtail](https://www.amazon.com/dp/B08JKRT68C) | The Orin Nano dev kit takes 9–20 V through a 5.5 × 2.5 mm jack. A full 5S battery is 21 V, so never connect the battery straight to the Jetson |
| 5S (18–20 V) power-tool battery, ≥ 4 Ah | 1 | [DeWalt 20V MAX 5 Ah (search)](https://www.amazon.com/dcb205/s?k=dcb205) | If you already own 18/20 V batteries of another brand, use those. You also need that brand's charger |
| Battery adapter with fuse and switch | 1 | [DeWalt 20V adapter, 12 AWG](https://www.amazon.com/dp/B0B9NPZM3M) | must match your battery brand |
| ATO fuse holders + 25 A, 5 A, 2 A fuses | 3 holders | [10 holders + 180 fuses, 1–40 A](https://www.amazon.com/dp/B09B8RR5BY) | |
| TVS diode, 15 V | 1 | [1.5KE15A, 20 pcs](https://www.amazon.com/dp/B079KJ5H9M) | The through-hole version is easier to wire than the SMBJ15A. Use the "A" (one-direction) type |
| Capacitor 2200 µF 25 V, low ESR | 1 | [10 pcs](https://www.amazon.com/dp/B0BFRDXQ1P) | |
| XT60 connector pair | 2 | [Amass, 1 pair](https://www.amazon.com/dp/B00RVM93LW) | buy 2 |

## Safety switches and relays

| item | need | buy | notes |
|---|---|---|---|
| 22 mm E-stop, latching, twist-release, 2 NC | 1 | [Amazon](https://www.amazon.com/dp/B07NNZB41H) | |
| 22 mm momentary push button, NO, flush (START) | 1 | [3-pack, 1 NO + 1 NC each](https://www.amazon.com/dp/B07GPJ6876) | use the green one and its NO contact |
| Relay K1: 12 V coil DPDT + socket | 1 | [MY2NJ with socket](https://www.amazon.com/dp/B0CPVL5PBN) | choose the **12 V DC** coil |
| Relay K2: 12 V 40 A automotive + socket | 1 | [2-pack with pre-wired sockets](https://www.amazon.com/dp/B07MR3KSWX) | |
| Roller-lever microswitch (SS-5GL size) | 1 | [Omron SS-5GL2, 5-pack](https://www.amazon.com/dp/B07FZBZGZD) | dock interlock |

## Wiring

| item | need | buy | notes |
|---|---|---|---|
| GX16-5 connector pair | 1 | [Amazon](https://www.amazon.com/dp/B07D235H6Z) | |
| GX12-4 connector pair | 1 | [2 sets](https://www.amazon.com/dp/B07V554X78) | |
| Silicone wire 14 AWG, red + black | 2 m each | [20 ft](https://www.amazon.com/dp/B08Q43DNKB) | |
| Silicone wire 22 AWG, 3 colours | 5 m | [6-colour kit](https://www.amazon.com/dp/B07WYYDBZP) | |
| Extra-flexible 3-core 26 AWG silicone cable | 2 × 0.5 m | [Amazon](https://www.amazon.com/dp/B0CJJF61V6) | choose **26 AWG, 3-pin** |
| WAGO 221 lever connectors | 1–2 | [search: WAGO 221-414](https://www.amazon.com/wago-221-414/s?k=wago+221-414) | |
| Spiral wrap 6 mm | 1 | [Amazon](https://www.amazon.com/dp/B01MG8H0ED) | |
| Cable ties 2.5 mm | 1 pack | [Amazon](https://www.amazon.com/dp/B000RB2DRK) | |
| PG7 cable gland | 1 | [20-pack](https://www.amazon.com/dp/B08TCFM4C7) | |

## Bearings

| item | need | buy |
|---|---|---|
| 6806-2RS, 30 × 42 × 7 | 2 | [2-pack](https://www.amazon.com/dp/B07TV2HVZ3) |
| 608-2RS, 8 × 22 × 7 | 6 | [10-pack](https://www.amazon.com/dp/B01M62EBAX) |
| 6802-2RS, 15 × 24 × 5 | 4 | [10-pack](https://www.amazon.com/dp/B0DL11HPGC) |
| 6800-2RS, 10 × 19 × 5 | 2 | [5-pack](https://www.amazon.com/dp/B08K2SR3CR) |
| 626-2RS, 6 × 19 × 6 | 2 | [10-pack](https://www.amazon.com/dp/B071JTPQN6) |
| 623-2RS, 3 × 10 × 4 | 3 | [10-pack](https://www.amazon.com/dp/B0DXKJDTM4) |
| 6808-2RS, 40 × 52 × 7 | 2 | [4-pack](https://www.amazon.com/dp/B08CR72M1X) |
| 6805-2RS, 25 × 37 × 7 | 2 | [2-pack](https://www.amazon.com/dp/B0DKZRQHXJ) |
| 688-2RS, 8 × 16 × 5 | 4 | [10-pack](https://www.amazon.com/dp/B0854DZNCJ) |

## Rail and belts

| item | need | buy | notes |
|---|---|---|---|
| MGN9 rail, cut to 124 mm, + 2 MGN9C carriages | 1 rail, 2 carriages | [2 × 150 mm rails + 4 MGN9C carriages](https://www.amazon.com/dp/B0H33B1SPS) | Cut one rail to 124 mm and deburr it. Keep the carriages on their plastic strip until they go on the rail, or the balls fall out |
| HTD belt 240-3M-15 | 1 | [Amazon](https://www.amazon.com/dp/B07R6HNJ5S) | |
| HTD belt 345-3M-15 | 1 | [Amazon](https://www.amazon.com/dp/B08R9XCWPX) | choose **345 mm, 15 mm wide** |
| HTD belt 321-3M-15 | 1 | [Amazon](https://www.amazon.com/dp/B01ESWL5T0) | |

## Metal stock (cut to length)

| item | need | buy | notes |
|---|---|---|---|
| Aluminium tube 12 × 1 mm | cut 2 × 182, 2 × 83, 1 × 15.7 mm | [12 OD × 10 ID × 300 mm, 4 pcs](https://www.amazon.com/dp/B09MHJ824D) | |
| Aluminium tube 10 × 1 mm | cut 222 mm | [10 OD × 8 ID × 250 mm](https://www.amazon.com/dp/B0B4W5YBYQ) | |
| Aluminium tube 8 × 1 mm | cut 53.5 mm | [8 OD × 6 ID × 300 mm, 3 pcs](https://www.amazon.com/dp/B0DNZ7BZKY) | |
| 15 mm hardened shaft | cut 94 mm | [15 × 500 mm](https://www.amazon.com/dp/B08K5JB213) | hardened: use an abrasive cut-off wheel |
| 10 mm hardened shaft | cut 56.2 mm | [2 × 10 × 100 mm](https://www.amazon.com/dp/B0CW8P9JG3) | same |

The BOM asks for h6 shafts; most Amazon shafts are h8 or unspecified. That's fine here, because the fit-test coupon (`00`) sets the printed hole sizes.

## Pins, shoulder screws, collars and shims

| item | need | buy | notes |
|---|---|---|---|
| Dowel pin 8 × 24 | 3 | [10-pack](https://www.amazon.com/dp/B009TE3RMG) | |
| Dowel pin 8 × 30 | 2 | [10-pack](https://www.amazon.com/dp/B009TE3T4W) | |
| Dowel pin 3 × 12 | 2 | [100-pack](https://www.amazon.com/dp/B009TE04AO) | |
| Dowel pin 2 × 20 (or an M2 × 20 screw + nut) | 1 | [50-pack](https://www.amazon.com/dp/B089N5QS96) | |
| Dowel pin 6 × 20 | 2 | [hardened, 25-pack](https://www.amazon.com/dp/B07CVN9KRD) | dock locating pins |
| Dowel pin 6 × 12 | 1 | [hardened, 25-pack](https://www.amazon.com/dp/B07CVNWYW6) | J1 hard stop |
| Dowel pin 4 × 8 | 1 | [100-pack](https://www.amazon.com/dp/B009TE3DDE) | |
| Shoulder screw, 6 mm × 10 mm shoulder, M5 thread | 2 | [25-pack](https://www.amazon.com/dp/B01MY9X3HL) | |
| Shaft collar, 30 mm bore | 1 | **on hold** | see below |
| Shaft collar, 15 mm bore, ≤ 28 mm OD, ≤ 12 mm wide | 1 | [2-pack, 24 mm OD × 10 mm](https://www.amazon.com/dp/B0C9QWCKC9) | |
| Shaft collar, 10 mm bore, ≤ 20 mm OD, ≤ 8 mm wide | 1 | [4-pack, 16 mm OD × 8 mm](https://www.amazon.com/dp/B0D2BHQDR9) | |
| Shim rings (DIN 988) | 15 × 21 in 0.1 / 0.3 / 0.5 / 1.0 mm; 10 × 16 × 0.5 mm (2); 8 × 14 in 0.1 / 0.3 mm (4) | [one listing; choose size and thickness](https://www.amazon.com/dp/B0DKQFJQZB) | one order per size/thickness |

## Cord, consumables and mount hardware

| item | need | buy | notes |
|---|---|---|---|
| Gravity-balance extension springs | 4 | **on hold** | see below |
| Dyneema cord 1.5–2 mm, ≥ 100 kg | 1 m | [2 mm, 400 kg](https://www.amazon.com/dp/B0DM5G55BC) | |
| M4 eye bolts, 40 mm, + nuts | 4 | [10-pack with nuts](https://www.amazon.com/dp/B0CM8SCKV1) | check that the threaded part is about 30 mm long |
| Silicone rubber sheet 1.5 mm, Shore A 40–60 | 1 | [50A, 1/16 in (1.6 mm)](https://www.amazon.com/dp/B08DDFQ9YM) | |
| Glue for the silicone facing | 1 | [Loctite 406](https://www.amazon.com/dp/B00B3HZPWC) + [Loctite 770 primer](https://www.amazon.com/dp/B0839PH46V) | the primer is what makes the glue stick to silicone |
| PTFE grease (plastic-safe) | 1 | [Super Lube 21030](https://www.amazon.com/dp/B07Q15DMBW) | |
| Knurled thumb screw M6 × 25 (DIN 464 style) | 2 | [Kipp, 10-pack](https://www.amazon.com/dp/B013IA7XWQ) | |
| O-ring that keeps each thumb screw captive | 2 | [5 mm ID × 1.5 mm, 50-pack](https://www.amazon.com/dp/B071JQ12YQ) | It must grip the M6 thread. The BOM used to say 13 × 1.5, which is the recess size; corrected |
| Rod end SI5, M5 female | 2 | [4-pack with jam nuts](https://www.amazon.com/dp/B0D2H88CLD) | |
| M5 threaded rod + 2 jam nuts | 1 | [M5 × 250 mm, right-hand, 2-pack](https://www.amazon.com/dp/B07ZNNPQMZ) + [jam nuts](https://www.amazon.com/dp/B00U925T6O) | Must be **right-hand** thread to match the rod ends. The rod ends above already include jam nuts |
| Rubber liner for the tube clamps | 5 | free | strips cut from an old bike inner tube, or print TPU sleeves |
| Safety tether: 1.5 mm wire-rope loop + snap hook | 1 | [1.5 mm × 60 cm loops, 4 pcs](https://www.amazon.com/dp/B0CZP8JRM8) + [stainless snap hooks, 2 pcs](https://www.amazon.com/dp/B07HP27WPZ) | |

## Heat-set inserts

| item | need | buy | notes |
|---|---|---|---|
| M3 × 5.7 (4.0 mm hole) | 52 | [ruthex, 100 pcs](https://www.amazon.com/dp/B08BCRZZS3) | |
| M4 × 8.1 (5.6 mm hole) | 27 | [ruthex, 50 pcs](https://www.amazon.com/dp/B07YSV66Y5) | |
| M5 × 9.5, 7 mm OD (6.4 mm hole) | 16 | [50 pcs](https://www.amazon.com/dp/B0CZXK17H7) | |
| M6 × 12.7 (8.0 mm hole) | 6 | [search: ruthex M6x12.7](https://www.amazon.com/s?k=ruthex+M6x12.7) | I didn't find a clear Amazon listing; get ones made for an 8.0 mm hole |

## On hold

1. **Gravity-balance extension springs (4).**
   - The rate spec in `BOM.md` and docs/05 §3 is right for balancing the arm:
     - J2 springs: 1.12 N/mm (6.4 lb/in), at least 114 N (25.6 lbf), at least 70 mm stretch, initial tension under 10 N.
     - J3 springs: 1.55 N/mm (8.9 lb/in), at least 137 N (30.8 lbf), at least 50 mm stretch, initial tension under 22 N.
   - A spring that meets this is about **12–16 mm in diameter and 85–100 mm long unstretched**, and up to about 180 mm long when stretched. The CAD only has a 9 mm-wide placeholder, so where these springs mount still has to be designed.
   - Don't buy them until that's done. Suppliers that sell springs by rate: [Century Spring](https://www.centuryspring.com/resources/how-to-select-an-extension-spring) and McMaster-Carr.
2. **J1 shaft collar, 30 mm bore (1).** The base has room for a collar 45 mm across and 13 mm wide. Standard 30 mm collars are 16 mm wide, such as the Ruland MSC-30 and Climax MC-30, or more than 45 mm across, so none fits as drawn. The fix is a design change: either a printed clamp collar, or a slightly taller base cavity for a standard 16 mm one.
