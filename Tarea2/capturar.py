"""Interactive serial CSV recorder. Never starts motion on its own."""
import argparse
import csv
import datetime
from pathlib import Path
import queue
import threading
import time

def main():
    import serial
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='COM4')
    parser.add_argument('--salida',type=Path,default=Path('ensayo_OA.csv'))
    args=parser.parse_args()
    if args.salida.exists() or args.salida.with_suffix('.txt').exists():
        raise SystemExit('Ya existe el registro. Usa otro nombre para conservarlo.')
    pending=queue.Queue()
    def keyboard():
        while True:
            try: value=input()
            except EOFError: value='SALIR'
            pending.put(value)
            if value.strip().upper()=='SALIR':return
    # Set control lines BEFORE opening; do not deliberately reset the board.
    port=serial.Serial();port.port=args.port;port.baudrate=115200;port.timeout=.1
    port.dtr=False;port.rts=False
    with port, args.salida.open('x',newline='',encoding='utf-8') as out, args.salida.with_suffix('.txt').open('x',encoding='utf-8') as raw:
        writer=csv.writer(out);header=None
        raw.write('# Captura '+datetime.datetime.now().isoformat()+'\n')
        print('Conectado. H ayuda, ? estado. Cada orden con Enter. SALIR termina captura.')
        threading.Thread(target=keyboard,daemon=True).start()
        try:
            while True:
                while not pending.empty():
                    cmd=pending.get()
                    if cmd.strip().upper()=='SALIR':return
                    port.write((cmd+'\n').encode('ascii',errors='strict'))
                    raw.write('# ENVIADO '+cmd+'\n');raw.flush()
                line=port.readline().decode('utf-8',errors='replace').strip()
                if not line:continue
                print(line);raw.write(line+'\n');raw.flush()
                if line.startswith('t_ms,'):
                    new_header=next(csv.reader([line]))
                    if header is None:header=new_header;writer.writerow(header)
                    elif header!=new_header:raise RuntimeError('Cambio de formato CSV')
                elif header and line[0].isdigit():
                    row=next(csv.reader([line]))
                    if len(row)==len(header):writer.writerow(row);out.flush()
        finally:
            # An explicit stop on exit; never auto-start the bench.
            port.write(b'S\n');port.flush();time.sleep(.1)
            print('Se envio S; desconecta las fuentes al terminar.')

if __name__=='__main__':main()
