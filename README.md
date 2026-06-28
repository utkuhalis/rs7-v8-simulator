# 🏎️ Audi RS7 V8 Engine Simulator

Gerçek zamanlı, **prosedürel ses sentezi** ile çalışan bir Audi RS7 (4.0 TFSI biturbo V8)
motor & egzoz simülatörü. Hiçbir hazır ses dosyası kullanmaz — tüm motor sesi
matematiksel olarak (ateşleme darbeleri + rezonans filtreleri + patlama çekirdekleri)
gerçek zamanlı üretilir.

![Python](https://img.shields.io/badge/python-3.9%2B-blue)

## Özellikler

### 🔊 Ses
- **Cross-plane V8 karakteri** — ateşleme darbe treni + banka düzensizliği (lope/burble)
- **Rezonans + comb (waveguide)** — gerçek egzoz borusunun **metalik** tınısı
- **Gerçek patlama çekirdekleri** — gaz kesince "pat-pat-pat" + büyük **anti-lag BANG**
- **Turbo spool + blow-off** — yükselen ıslık ve gaz kesince "pışş" dump valf
- **Marş + söndürme + soğuk start** — gerçekçi çalıştırma/sönme, fast-idle, kaba rölanti
- **Rev-limiter sert kesme**, **rev-match blip**, **lastik cızırtısı**
- **Stereo + rüzgâr + ortam reverb** (Açık/Garaj/Tünel)
- **Drive-by / Doppler** modu, **EGZOZ / MOTOR** ses modu, **Stage 1/2/3** ses agresifliği

### 🏁 Fizik (gerçek RS7 C8 verileri)
- 800 Nm tork eğrisi (2050–4500 rpm düz), ~6800 devir limiti
- **Stage 1/2/3 tuning**: 0–100 ~3.5 / 3.0 / 2.6 / 2.3s
- Gerçek 8 ileri şanzıman, final drive, drag; üst hız **~300 km/h**, çeyrek mil ~11.6s
- **Launch control**, **patinaj**, **direksiyon + viraj + drift** (el freni)
- Motor freni, boş vites (N) + free-rev, fren, oto/manuel vites
- **Yakıt** tüketimi + **motor ısısı**, motor çalıştır/durdur

### 📊 HUD (Audi Virtual Cockpit tarzı)
- Analog devir saati + hız kadranı, F1 **shift light**, **G-metre** (boyuna + yanal)
- **Yan görünüm araç** (dönen jant, egzoz alevi, drift dumanı) + renk **temaları**
- **Telemetri grafiği** (rpm/hız/G), yakıt/ısı barları
- Canlı 0–100 / çeyrek mil + **kalıcı** en iyi dereceler (dosyaya kayıt)
- **Drag yarışı modu** (christmas tree + reaction time)

## Kurulum

```bash
pip install -r requirements.txt
python test.py
```

## Kontroller

| Tuş | İşlev |
|-----|-------|
| `Space` | Motoru çalıştır / durdur |
| `↑` / `Shift+↑` | Gaz / tam gaz |
| `↓` | Fren |
| `← / →` | Direksiyon |
| `B` | El freni (drift) |
| `Z / X` | Manuel vites düşür / yükselt |
| `1–8` | Direkt vites seç |
| `N` | Boş vites (free-rev) |
| `A` | Oto / Manuel vites |
| `M` | Motor / Egzoz ses modu |
| `T` | Stage (tuning) değiştir |
| `R` | Ortam akustiği (Açık/Garaj/Tünel) |
| `C` | Renk teması |
| `D` | Drive-by (yanından geçiş) |
| `G` | Drag yarışı (christmas tree) |
| `F` | Yakıt doldur |

> **Launch control:** Dururken `↓` fren + `Shift+↑` tam gaz ile devri tut, sonra freni bırak → fırla.

## Teknik

Ses motoru `sounddevice` ile kesintisiz callback stream üzerinden çalışır;
`scipy` IIR biquad rezonatörleri, overlap-add patlama sentezi ve `numpy`
vektörizasyonu kullanır. Fizik `pygame` ana döngüsünde gerçek tork/drag
modeliyle entegre edilir.

## Lisans

MIT
