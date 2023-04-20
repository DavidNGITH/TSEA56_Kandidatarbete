import winsound
import time

# Define the frequency of each note
C = 262
D = 294
E = 330
F = 349
G = 392
A = 440
B = 494

# Define the duration of each note
whole = 1600
half = 800
quarter = 400
eighth = 200

# Define the melody of Happy Birthday
melody = [(G, quarter), (G, quarter), (A, half), (G, half), (C, half), (B, whole),
          (G, quarter), (G, quarter), (A, half), (G, half), (D, half), (C, whole),
          (G, quarter), (G, quarter), (G, half), (E, half), (C, half), (B, whole),
          (F, quarter), (F, quarter), (E, half), (C, half), (D, half), (C, whole)]

# Play the melody
for note in melody:
    winsound.Beep(note[0], note[1])
    time.sleep(0.05)  # Add a small pause between each note
