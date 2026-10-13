"""Hilfen fuer Laeufe gegen die LeRobot-G1-Sim: Kamera-Port freiraeumen, Stillstand der Sim erkennen."""
import faulthandler
import os
import signal
import subprocess
import time

CAMERA_PORT = 5555  # ZMQ-Port, auf dem der Kamera-Prozess der Sim sendet


def port_owner(port=CAMERA_PORT):
    """(pid, ppid, Befehl) des Prozesses, der auf dem TCP-Port lauscht, sonst None."""
    pids = subprocess.run(["lsof", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN", "-t"],
                          capture_output=True, text=True).stdout.split()
    if not pids:
        return None
    pid = int(pids[0])
    ps = subprocess.run(["ps", "-p", str(pid), "-o", "ppid=,command="], capture_output=True, text=True).stdout.strip()
    ppid, _, command = ps.partition(" ")
    return pid, int(ppid or 0), command.strip()


def free_camera_port(port=CAMERA_PORT):
    """Port vor einem Lauf freiraeumen.

    Bricht ein Lauf ohne robot.disconnect() ab, bleibt der Kamera-Prozess der Sim als Waise zurueck (Elternprozess 1)
    und haelt den Port. Der naechste Lauf kann dann nicht senden. Nur solche Waisen werden beendet; gehoert der Port
    einem anderen Prozess, gibt es einen Fehler mit PID.
    """
    owner = port_owner(port)
    if owner is None:
        return
    pid, ppid, command = owner
    if ppid == 1 and "multiprocessing.spawn" in command:
        os.kill(pid, signal.SIGTERM)
        for _ in range(50):
            if port_owner(port) is None:
                print(f"verwaisten Kamera-Prozess beendet (PID {pid}), Port {port} frei")
                return
            time.sleep(0.1)
    raise RuntimeError(f"Port {port} ist belegt von PID {pid} ({command[:100]}). Pruefen und ggf. 'kill {pid}'.")


class StallWatch:
    """Erkennt, dass die Sim-Zeit laenger als `seconds` (Wanduhr) stillsteht.

    Beim ersten Stillstand werden die Stacks aller Threads nach stderr geschrieben - so ist sichtbar, wo die Sim
    haengt, statt dass ein Lauf nur "0,0 m" meldet.
    """

    def __init__(self, data, seconds=10.0):
        self.data = data
        self.seconds = seconds
        self.last_time = float(data.time)
        self.last_change = time.perf_counter()
        self.dumped = False

    def stalled(self):
        now = time.perf_counter()
        t = float(self.data.time)
        if t != self.last_time:
            self.last_time, self.last_change = t, now
            return False
        if now - self.last_change < self.seconds:
            return False
        if not self.dumped:
            print(f"SIM STEHT: Sim-Zeit seit {self.seconds:.0f} s unveraendert bei {t:.3f} s - Thread-Stacks:")
            faulthandler.dump_traceback(all_threads=True)
            self.dumped = True
        return True
