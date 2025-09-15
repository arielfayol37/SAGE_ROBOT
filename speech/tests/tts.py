import pyttsx3

# Initialize the engine
engine = pyttsx3.init()

# Set properties (optional, e.g., voice, rate, volume)
# voices = engine.getProperty('voices')
# engine.setProperty('voice', voices[1].id) # Change voice (index varies by system)
# engine.setProperty('rate', 150) # Speed of speech

# Add text to be spoken
engine.say("My name is Ariel Fayol. Why is this talking so weird man?")

# Process the speech and wait for it to finish
engine.runAndWait()

# Stop the engine (important for releasing resources)
engine.stop()