"""Publica un texto en un topico std_msgs/String y opcionalmente lo dice en voz alta
(espeak-ng dentro del Docker / Linux; voz de Windows si se corre en WSL)."""
import os
import shutil
import subprocess
import time


class Anunciador:
    def __init__(self, node=None, topic="/nombre_reconocido", voz=True, idioma="es"):
        self.pub = None
        if node is not None:
            from std_msgs.msg import String
            self.String = String
            self.pub = node.create_publisher(String, topic, 10)
        self.cmd = None
        if voz:
            ps = "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"
            if os.path.exists(ps):
                self.cmd = [ps, "-NoProfile", "-Command",
                            "Add-Type -AssemblyName System.Speech;"
                            "(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak($args[0])"]
            else:
                for exe, extra in (("espeak-ng", ["-v", idioma]), ("espeak", ["-v", idioma]),
                                   ("spd-say", ["-l", idioma, "-w"])):
                    if shutil.which(exe):
                        self.cmd = [exe, *extra]
                        break

    def anunciar(self, texto, decir=None):
        print(f"[anuncio] {texto}", flush=True)
        if self.pub is not None:
            for _ in range(3):
                self.pub.publish(self.String(data=texto))
                time.sleep(0.05)
        if self.cmd:
            subprocess.run(self.cmd + [decir or texto], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, check=False)
