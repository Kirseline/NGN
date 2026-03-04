# SENSORE


<img width="2620" height="952" alt="sensore" src="https://github.com/user-attachments/assets/cc90d550-4c26-4a49-a722-7c7c040a1031" />

Il flowgraph implementa il nodo sensore SDR multicanale basato su protocollo CSMA/CA e modulazione WiFi all'interno di una larghezza di banda globale di 10 MHz. L'acquisizione dati avviene tramite un parser CAN-USB che inietta i payload nel "controllore sensore", un blocco custom preposto alla gestione della macchina a stati MAC e alla valutazione dell'occupazione del canale calcolata dall'energia dei campioni in ricezione tramite media mobile e soglia. Il segnale OFDM in banda base viene generato dal blocco WiFi PHY a 1.25 MHz, successivamente espanso a 10 MHz tramite un Rational Resampler con fattore di interpolazione 8 e filtrato con un passa-basso per sopprimere le repliche spettrali. Lo shift sul canale è eseguito dinamicamente dal Frequency Xlating FIR Filter in trasmissione, il quale riceve i comandi asincroni dal controller per traslare il segnale su uno dei sei offset target predefiniti (−4.167e6, −2.500e6, −0.833e6, 0.833e6, 2.500e6, 4.167e6) prima dell'emissione sul mezzo fisico. Nel percorso di ricezione, il flusso a 10 MHz catturato dalla USRP Source viene processato da un filtro Xlating in decimazione (fattore 8) che isola il canale di interesse e lo riporta al campionamento necessario al blocco PHY WiFi, riportandolo in banda base per alimentare simultaneamente il decodificatore WiFi PHY, che estrapola i messaggi, e il ramo di rilevamento potenza essenziale per l'algoritmo di backoff e mitigazione delle collisioni.


## CAN-USB microncontroller

La funzione di interrupt CAN_RX_Handler gestisce la ricezione asincrona dei frame CAN, l'estrazione dei dati dai registri hardware a 32 bit del PSoC 5LP che funge da ponte tra il bus CAN e l'host, e la formattazione per l'interfaccia USBUART. Viene utilizzata una ISR generata dalla periferica CAN in cui si determina il DLC e l'ID, nonchè il payload. Prima di trasmettere i dati in USB, viene corretta la sequenza Little-Endian tramite un vettore di mappatura.

## CAN-USB Parser

Il blocco `CAN-USB parser` esegue la conversione di frame UART rappresentanti messaggi CAN in pacchetti pmt tipo vettore. Si utilizza il DLC dal CAN per gestire la lunghezza variabile del frame.

### Architettura del Frame

Il parsing richiede una struttura del messaggio seriale in ingresso rigida. La lunghezza totale del frame è `5 + N` byte, dove `N` è il valore intero letto dal campo DLC.

| Indice | Campo | Dimensione | Valore / Descrizione |
| :--- | :--- | :--- | :--- |
| `0` | **start ch** | 1 byte | `0x24`. Inizio validazione del frame |
| `1` | **ID high** | 1 byte | ID MSB  |
| `2` | **ID low** | 1 byte | ID LSB |
| `3` | **DLC** | 1 byte | Data Length Code. Definisce il numero di byte del payload (`N`). |
| `4` a `4 + N - 1` | **payload** | `N` byte | Dati raw |
| `4 + N` | **terminatore** | 1 byte | `0x0A`. Definisce la fine del messaggio |

### Logica operativa

Il parsing avviene in quattro stadi sequenziali iterati all'interno di un loop di 0.001s in un thread specifico.

### 1. Svuotamento asincrono del buffer seriale
L'intero contenuto disponibile nel buffer seriale viene trasferito in un `bytearray` locale ad ogni ciclo per svuotare il buffer UART.

```python
if self.ser and self.ser.in_waiting > 0:
    self.packet_buffer.extend(self.ser.read(self.ser.in_waiting))
```

### 2. Ricerca del possibile messaggio

Se il buffer contiene byte sufficienti per raggiungere il DLC (minimo 5 byte), il sistema analizza il byte in testa. Se non coincide con 0x24, viene distrutto, forzando lo spostamento della finestra di memoria in avanti fino al raggiungimento di una potenziale carattere iniziale valido.
``` python
while len(self.packet_buffer) >= 5: 
    if self.packet_buffer[0] != self.START_BYTE:
        self.packet_buffer.pop(0)
        continue
```

### 3. Calcolo Lunghezza dinamica

La lunghezza attesa viene estratta dal campo DLC. Questo previene la ricerca del terminatore, annullando il rischio di troncare i pacchetti qualora il valore 0x0A si presentasse casualmente nel payload.
``` python
dlc = self.packet_buffer[3]
expected_len = 5 + dlc
```

### 4. Validazione ed estrazione

Quando il buffer locale raggiunge o supera la dimensione attesa, il blocco interroga l'ultimo indice calcolato. Se il terminatore è assente, il pacchetto è corrotto e il byte di testa viene scartato per riavviare la ricerca di un messaggio valido. Se presente invece, estrae dalla memoria il frame senza i byte di controllo (0x24 e 0x0A), lo converte in u8vector tramite le librerie PMT e libera la memoria processata.
``` python
if len(self.packet_buffer) >= expected_len:
    if self.packet_buffer[expected_len - 1] == self.TERMINATOR:
    
        frame = list(self.packet_buffer[1:expected_len - 1]) # estrae frame escluso start e terminatore
        
        msg = pmt.init_u8vector(len(frame), frame)
        self.message_port_pub(pmt.intern("out"), msg) # inoltro sulla porta di uscita
        
        self.packet_buffer = self.packet_buffer[expected_len:]
    else:
        self.packet_buffer.pop(0)
```

## Sensor State Controller

Il modulo implementa una macchina a stati finiti per la gestione dell'accesso tramite protocollo CSMA/CA semplificato e la riallocazione dinamica della frequenza. La logica operativa è divisa tra un thread sincrono di elaborazione campioni (work) per la valutazione del canale (CCA) e un thread asincrono (run_loop) per le transizioni di stato.


<img width="1566" height="1418" alt="sensor state controller fsm" src="https://github.com/user-attachments/assets/8f636241-9320-4f51-ad55-e21ad826f5da" />


### 1. Valutazione CCA (Clear Channel Assessment)

La funzione work monitora costantemente l'energia del canale elaborando i campioni in ingresso. Aggiorna un flag globale utilizzato dalla macchina a stati.

``` python 
def work(self, input_items, output_items):
        if len(input_items[0]) > 0:
            valore_grezzo = input_items[0][-1]
            self.channel_is_free = (valore_grezzo < 0.5)
            self.last_sample_time = time.time() 
        return len(input_items[0])
```
### 2. Stato REQ (Fase di Negoziazione)

Il thread run_loop interroga il flag CCA. Se il canale è libero, trasmette una richiesta di associazione (REQ_ID). Se occupato, innesca un backoff randomico per mitigare le collisioni. Il superamento delle soglie massime forza lo stato ERROR (max retry).
``` python 
if self.current_state == "REQ":
  if self.req_attempts >= self.max_retries:
    self.current_state = "ERROR"
    continue

  if self.channel_is_free:
    self.req_attempts += 1
    self.backoff_attempts = 0
    # messaggi negoziazione "REQ_ID"
    req_msg = f"REQ_{self.sensor_id}"
    raw_payload = list(req_msg.encode('utf-8'))                
    meta = pmt.make_dict()
    data = pmt.init_u8vector(len(raw_payload), raw_payload)
    self.message_port_pub(pmt.intern("pdu_out"), pmt.cons(meta, data))

    time.sleep(self.retry_delay_ms / 1000.0)
  else:
    if self.backoff_attempts >= self.max_backoff_attempts:
      self.current_state = "ERROR"
      continue
    self.backoff_attempts += 1
    backoff = random.uniform(0, self.max_backoff_ms)
    time.sleep(backoff / 1000.0)
```
### 3. Transizione REQ → SIG (Gestore Messaggi)
Il gestore asincrono handle_pdu intercetta le risposte dell'Hub. Estrae l'indice del canale di destinazione dal payload e forza la transizione di stato, spostando l'offset di frequenza del trasmettitore.
``` python 
def handle_pdu(self, msg):
        if self.current_state != "REQ":
            return
        
        #estrazione dati payload dal MAC wifi...
        decoded_str = bytes(payload_bytes[24:]).decode('latin-1').strip()
        
        if decoded_str.startswith("REQ") and f"_{self.sensor_id}" in decoded_str:
            ch_part = decoded_str.split('_')[0]
            ch_index = int("".join(filter(str.isdigit, ch_part)))
            
            if 0 <= ch_index < len(self.offsets):
                # Transizione di stato 
                self.current_channel = ch_index
                self.current_state = "SIG"
               
```

### 4. Stato SIG (Fase Operativa)

Con lo stato settato su SIG, il controller agisce come un gate aperto. La funzione handle_payload converte i dati grezzi in ingresso in messaggi PDU diretti al blocco PHY, trasmettendo sul nuovo offset di frequenza assegnato.
``` python 
def handle_payload(self, msg):
        # Blocco trasmissione dati se non in stato SIG
        if self.current_state != "SIG":
            return

        if pmt.is_u8vector(msg):
            raw_payload = list(pmt.u8vector_elements(msg))
            meta = pmt.make_dict()
            data = pmt.init_u8vector(len(raw_payload), raw_payload)
            self.message_port_pub(pmt.intern("pdu_out"), pmt.cons(meta, data))
```


# HUB

<img width="2752" height="1402" alt="hub controller" src="https://github.com/user-attachments/assets/0d8cc25b-1a11-4cb7-b38e-8b353adf4558" />


L'architettura di rete si completa con il nodo centralizzato HUB, strutturato per fungere da gateway verso un host e coordinatore dell'intera comunicazione radio. A differenza del sensore che rilocava dinamicamente un singolo ricetrasmettitore, l'Hub implementa una demultiplazione parallela in ricezione: il flusso continuo a 10 MHz proveniente dalla USRP Source viene sdoppiato in sei rami indipendenti. Ogni ramo impiega un Frequency Xlating FIR Filter dedicato, configurato con uno dei sei offset fissi e decimazione 8, per isolare il rispettivo canale e riportarlo in banda base. Questo design alimenta sei blocchi WiFi PHY simultanei, permettendo la decodifica concorrente dei payload provenienti dai nodi sensore allocati in stato operativo SIG. Il controllo degli accessi è governato dal blocco "HUB_Controller", che ascolta le richieste di unione REQ sul canale primario di negoziazione e gestisce la disponibilità delle frequenze. Le risposte all'assegnazione del canale vengono modulate da una catena di trasmissione dedicata (WiFi MAC -> PHY -> interpolatore hardware-software e traslatore di frequenza) e spedite tramite la USRP Sink ai sensori in attesa, chiudendo l'anello di controllo asincrono della rete.

## HUB State Controller

<img width="1885" height="1460" alt="fsm hub" src="https://github.com/user-attachments/assets/9d8839f1-9f56-42a8-8051-3c8c0647796a" />

Il blocco inizializza un pool di canali liberi e un registro temporale per tracciare le assegnazioni attive. Questa struttura previene l'esaurimento delle risorse causato da richieste dallo stesso ID in rapida succesione.
La funzione handle_msg decodifica i pacchetti in ingresso. Scarta i primi 24 byte, corrispondenti all'header MAC del protocollo 802.11, e isola il payload. L'identificativo del sensore viene estratto isolando il suffisso numerico dalla stringa REQ_ID.
``` python 
if len(payload_bytes) > 24:
    decoded_str = bytes(payload_bytes[24:]).decode('latin-1').strip()
    if decoded_str.startswith("REQ"):
        parts = decoded_str.split('_')
        sensor_id = parts[1] if len(parts) > 1 else "0"
```

### Logica Anti-Burn e Ritrasmissione

In presenza di latenze o perdite di pacchetti di conferma, un sensore potrebbe iterare la richiesta iniziale. L'Hub previene l'assegnazione multipla interrogando active_assignments. Se la richiesta dello stesso sensor_id avviene entro la finestra di cooldown, il controllore ritrasmette il canale precedentemente allocato senza alterare la lista dei canali liberi.
``` python 
if sensor_id in self.active_assignments:
    assigned_ch, last_time = self.active_assignments[sensor_id]
    if (now - last_time) < self.cooldown:
        reply_str = f"REQ{assigned_ch}_{sensor_id}"
        self._send_reply(reply_str)
        return
```

### Allocazione Nuovo Canale

Se la richiesta è nuova o il cooldown è scaduto, il sistema estrae il primo elemento disponibile dall'array free_channels. Il dizionario di stato viene aggiornato con il nuovo vincolo ID-Canale e il timestamp corrente. La stringa formatta l'istruzione di salto in frequenza.
``` python 
if self.free_channels:
    assigned_ch = self.free_channels.pop(0)
    self.active_assignments[sensor_id] = (assigned_ch, now)
    reply_str = f"REQ{assigned_ch}_{sensor_id}"
    self._send_reply(reply_str)
```


[#] Parser Seriale CAN avviato su /dev/ttyACM0
[1] SENSORE 1 f -4167000.0 CCA libero - TX REQ 1/5
[1] SENSORE 1 f -4167000.0 CCA libero - TX REQ 2/5
[1] HUB ricevuto: REQ_1
[2] HUB assegnazione: REQ3_1
[1] SENSORE 1 f -4167000.0 CCA libero - TX REQ 3/5
[1] HUB ricevuto: REQ_1
[RE-TX] HUB ripropone REQ3_1 per ID 1
[2] SENSORE 1 RX VALIDO: REQ3_1
[3] SENSORE 1 SALTO su f-833000.0
[4] SENSORE 1 f 833000.0 TX raw [0, 86, 4, 34, 51, 68, 0]
HUB parser CH3 raw : [0, 86, 4, 34, 51, 68, 0]
[4] SENSORE 1 f 833000.0 TX raw [0, 86, 4, 34, 51, 68, 0]
HUB parser CH3 raw : [0, 86, 4, 34, 51, 68, 0]
[4] SENSORE 1 f 833000.0 TX raw [0, 86, 4, 34, 51, 68, 0]
HUB parser CH3 raw : [0, 86, 4, 34, 51, 68, 0]
[4] SENSORE 1 f 833000.0 TX raw [0, 86, 4, 34, 51, 68, 0]
HUB parser CH3 raw : [0, 86, 4, 34, 51, 68, 0]
[4] SENSORE 1 f 833000.0 TX raw [0, 35, 4, 1, 2, 3, 0]
HUB parser CH3 raw : [0, 35, 4, 1, 2, 3, 0]
