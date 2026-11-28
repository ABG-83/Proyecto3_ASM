"""Interactive serial CSV recorder. Never starts motion on its own."""
import argparse
import csv
import datetime
from pathlib import Path
import queue
import threading
import time

ENCABEZADO = 't_ms,modo,ref,w,e,u,pwm,nueva'.split(',')


def guardar_linea(linea, salida, encabezado_recibido):
    # Los eventos se conservan en el CSV como comentarios para MATLAB.
    if linea.startswith('#'):
        salida.write(linea + '\n')
    elif linea.startswith('t_ms,'):
        if next(csv.reader([linea])) != ENCABEZADO:
            raise ValueError('El encabezado CSV no coincide con Registro.h.')
        if encabezado_recibido:
            raise ValueError('Usa un archivo distinto para cada ensayo.')
        csv.writer(salida).writerow(ENCABEZADO)
        encabezado_recibido = True
    elif encabezado_recibido and linea[0].isdigit():
        # Un registro incompleto queda en el TXT y se avisa en consola.
        fila = next(csv.reader([linea]))
        if len(fila) == len(ENCABEZADO) and fila[0].isascii() and fila[0].isdigit():
            csv.writer(salida).writerow(fila)
        else:
            print('Fila CSV incompleta o inválida. Consulta el registro TXT.')
    return encabezado_recibido


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
        encabezado_recibido = False
        fragmentos = bytearray()
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
                # Una lectura con timeout puede terminar en mitad de una fila.
                fragmento = port.readline()
                fragmentos.extend(fragmento)
                if not fragmento.endswith(b'\n'):
                    continue
                line = fragmentos.decode('utf-8', errors='replace').strip()
                fragmentos.clear()
                if not line:continue
                print(line);raw.write(line+'\n');raw.flush()
                encabezado_recibido = guardar_linea(line, out, encabezado_recibido)
                out.flush()
        finally:
            # An explicit stop on exit; never auto-start the bench.
            port.write(b'S\n');port.flush();time.sleep(.1)
            print('Se envio S; desconecta las fuentes al terminar.')

if __name__=='__main__':main()
