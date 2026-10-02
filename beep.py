from gpiozero import TonalBuzzer
from gpiozero.tones import Tone
from time import sleep

buzzer1 = TonalBuzzer(17)
buzzer2 = TonalBuzzer(27)


def melodie_victoire():
    global buzzer1, buzzer2
    partition = [
        ("C5", "E5", 0.1),
        ("C5", "E5", 0.1),
        ("C5", "E5", 0.1),
        ("C5", "E5", 0.3),
        ("G4", "C5", 0.3),
        ("Bb4", "D5", 0.3),
        ("C5", "E5", 0.2),
        ("Bb4", "D5", 0.1),
        ("C5", "E5", 0.6)
    ]
    for note1, note2, duree in partition:
        buzzer1.play(Tone(note1))
        buzzer2.play(Tone(note2))
        sleep(duree)
        buzzer1.stop()
        buzzer2.stop()
        sleep(0.05)

def bip_changement_couleur(nombre):
    print("beep")

# --- Exemple d'utilisation dans ton script principal ---
if __name__ == "__main__":

    # Appel au moment de changer de couleur
    bip_changement_couleur(2)
    sleep(1)

    # Appel à la fin du parcours
    melodie_victoire()
