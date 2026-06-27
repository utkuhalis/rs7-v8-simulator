# 🏎️ Audi RS7 V8 Engine Simulator

Gerçek zamanlı, **prosedürel ses sentezi** ile çalışan bir Audi RS7 (4.0 TFSI biturbo V8)
motor & egzoz simülatörü. Hiçbir hazır ses dosyası kullanmaz — tüm motor sesi
matematiksel olarak (ateşleme darbeleri + rezonans filtreleri + patlama çekirdekleri)
gerçek zamanlı üretilir.

![Python](https://img.shields.io/badge/python-3.9%2B-blue)

## Özellikler

### 🔊 Ses
- **Cross-plane V8 karakteri** — ateşleme darbe treni + banka düzensizliği (lope/burble)
- **Rezonans tabanlı egzoz** — gerçek egzoz borusu rezonansları (saf sinüs değil)
- **Gerçek patlama çekirdekleri** — gaz kesince "pat-pat-pat" overrun çıtırtısı
- **Sub-bass thump** — göğüste hissedilen alt frekanslar
- **EGZOZ / MOTOR ses modu** — dışarıdan vs kabin içi dinleme

### 🏁 Fizik (gerçek RS7 C8 verileri)
- 800 Nm tork eğrisi (2050–4500 rpm düz), ~6800 devir limiti
- Gerçek 8 ileri şanzıman oranları, final drive, drag modeli
- **0–100 km/h ~3.9s**, üst hız **~300 km/h**
- Motor freni, boş vites (N) + free-rev, fren, oto/manuel vites

## Kurulum

```bash
pip install -r requirements.txt
python test.py
```

## Kontroller

| Tuş | İşlev |
|-----|-------|
| `↑` | Gaz (kısmi, feather edilebilir) |
| `Shift + ↑` | Tam gaz |
| `↓` | Fren |
| `A` | Oto / Manuel vites |
| `M` | Motor / Egzoz ses modu |
| `N` | Boş vites (free-rev) |
| `← / →` | Vites düşür / yükselt (manuel) |
| `1–8` | Direkt vites seç |

## Teknik

Ses motoru `sounddevice` ile kesintisiz callback stream üzerinden çalışır;
`scipy` IIR biquad rezonatörleri, overlap-add patlama sentezi ve `numpy`
vektörizasyonu kullanır. Fizik `pygame` ana döngüsünde gerçek tork/drag
modeliyle entegre edilir.

## Lisans

MIT
