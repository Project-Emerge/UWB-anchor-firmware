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
superframe. Sono configurati staticamente quattro ID di ancora (`A001`–`A004`)
e dodici ID di tag (`B001`–`B00C`). La configurazione usa canale 5, 6.8 Mbps,
PRF 64 MHz e preambolo da 64 simboli.

Il superframe contiene uno slot DS-TWR per ogni combinazione ancora/tag, quindi
il rate di aggiornamento è determinato **dalle dimensioni delle tabelle** prima
ancora che dai tempi: ogni nodo previsto ma non installato costa comunque uno
slot pieno. Con quattro ancore e dodici tag sono 48 slot da 5.5 ms, cioè 274 ms
di superframe → circa **3.6 Hz per tag**.

Il limite non è il tempo di volo ma il turnaround SPI/MCU tra la ricezione di un
frame e la programmazione della risposta. Per questo il core gira a 80 MHz e
l'SPI passa a 20 MHz subito dopo l'init del DW3000 (il vincolo <7 MHz vale solo
in INIT_RC). Per salire ancora, la leva più efficace è ridurre `TAG_IDS` al
numero di robot realmente in campo: a parità di tempi, 6 tag danno ~7 Hz e 4 tag
~10 Hz. La tabella completa è nel commento di `SUPERFRAME_US` in `src/uwb.rs`.

## Build e identificativi

L'ID è selezionato in compilazione. L'ancora `anchor-1` è sempre il master:

```console
cargo build --release -p uwb-anchor --no-default-features --features anchor-1
cargo build --release -p uwb-anchor --no-default-features --features anchor-2
cargo build --release -p uwb-tag --no-default-features --features tag-1
```

Le feature disponibili sono `anchor-1` … `anchor-4` e `tag-1` … `tag-12`.
Le tabelle ID e i parametri TDMA sono in `src/uwb.rs`.

## Messa in servizio

Il reset del DWM3000 viene pilotato open-drain: PA1 è trascinato low e poi
rilasciato in input. Non deve essere forzato high.

I ritardi antenna (`TX_ANTENNA_DELAY` / `RX_ANTENNA_DELAY` in `src/uwb.rs`) sono
impostati a 16385 tick, il valore nominale degli esempi Qorvo DW3000 per la
configurazione a 64 MHz PRF usata qui. Non è un valore calibrato: il ritardo
reale dipende da modulo e layout, quindi resta un bias costante che può valere
alcune decine di centimetri. Per calibrare, misurare una distanza nota e
correggere il valore: un tick vale circa 4,69 mm e, poiché il ritardo si applica
a entrambi i capi del collegamento, variare di N tick sposta la distanza
riportata di circa 2 * N tick. Entrambi i nodi devono usare gli stessi valori,
per questo sono condivisi nella libreria invece di essere duplicati.

Le coordinate delle ancore restano nel firmware del robot, che associa le
distanze agli ID statici e svolge la trilaterazione.

La gestione esistente di bootstrap STM6600, monitor batteria, LED e BQ24074 è
preservata. Il mapping dei LED è stato corretto: PA8 è rosso e PA9 è verde.
