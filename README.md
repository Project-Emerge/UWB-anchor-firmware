# Firmware DWM3000 per ancore e tag

Questo workspace contiene il firmware STM32L432 per le ancore e un tag
temporaneo, entrambi con modulo Qorvo DWM3000 e driver
[`dw3000-ng`](https://crates.io/crates/dw3000-ng).

## Ranging e capacità

Il sistema usa asymmetric double-sided two-way ranging (DS-TWR):

1. L'ancora invia un `Poll` nello slot assegnato al tag.
2. Il tag risponde con `Request` a trasmissione ritardata.
3. L'ancora invia `Response` con i timestamp hardware necessari; il tag
   calcola e registra la distanza in millimetri.

L'ancora `A001` è il master TDMA e sincronizza le altre ancore via UWB a ogni
superframe. Sono configurati staticamente cinque ID di ancora (`A001`–`A005`)
e dodici ID di tag (`B001`–`B00C`). La configurazione usa canale 5, 6.8 Mbps,
PRF 64 MHz e preambolo da 64 simboli.

Con cinque ancore e dodici tag il superframe è 130 ms, cioè circa **7.7 Hz per
tag**. È la scelta affidabile con DS-TWR e slot da 2 ms; 12 robot a 20 Hz
richiederebbero 60 ranging completi ogni 50 ms e non lascerebbero margine per
ritardi SPI, retry o propagazione multipath. Per quattro ancore il timing può
essere ridotto dopo validazione hardware.

## Build e identificativi

L'ID è selezionato in compilazione. L'ancora `anchor-1` è sempre il master:

```console
cargo build --release -p uwb-anchor --no-default-features --features anchor-1
cargo build --release -p uwb-anchor --no-default-features --features anchor-2
cargo build --release -p uwb-tag --no-default-features --features tag-1
```

Le feature disponibili sono `anchor-1` … `anchor-5` e `tag-1` … `tag-12`.
Le tabelle ID e i parametri TDMA sono in `src/uwb.rs`.

## Messa in servizio

Il reset del DWM3000 viene pilotato open-drain: PA1 è trascinato low e poi
rilasciato in input. Non deve essere forzato high.

I ritardi antenna sono impostati provvisoriamente a zero per rendere esplicita
la necessità di calibrazione per ogni coppia di layout/modulo. Prima di usare
le misure per trilaterazione occorre calibrare TX/RX antenna delay e inserire
i valori nella fase di inizializzazione di ancora e tag. Le coordinate delle
ancore restano nel firmware del robot, che associa le distanze agli ID statici
e svolge la trilaterazione.

La gestione esistente di bootstrap STM6600, monitor batteria, LED e BQ24074 è
preservata. Il mapping dei LED è stato corretto: PA8 è rosso e PA9 è verde.
