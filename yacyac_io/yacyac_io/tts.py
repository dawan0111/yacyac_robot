#!/usr/bin/env python3

"""Synthesizes speech from the input string of text or ssml.
Make sure to be working in a virtual environment.

Note: ssml must be well-formed according to:
    https://www.w3.org/TR/speech-synthesis/
"""
from google.cloud import texttospeech
import os
import playsound

import rclpy as rp
import time as time
from rclpy.node import Node
from yacyac_interface.srv import TTS


class SpeakTTS(Node):
    def __init__(self):
        super().__init__("yacyac_io")
        self.is_speak = False
        self.srv = self.create_service(TTS, '/yacyac/io', self.callback_service)
        # Instantiates a client
        self.client = texttospeech.TextToSpeechClient()
        # Build the voice request, select the language code ("en-US") and the ssml
        # voice gender ("neutral")
        self.voice = texttospeech.VoiceSelectionParams(
            language_code="ko-KR", ssml_gender=texttospeech.SsmlVoiceGender.FEMALE
        )
        # Select the type of audio file you want returned
        self.audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3
        )    

    def callback_service(self, request, response):
        response.tts_str_s = request.tts_str_t

        # Set the text input to be synthesized
        text = str(response.tts_str_s)
        synthesis_input = texttospeech.SynthesisInput(text=text)

        self.response = self.client.synthesize_speech(
            input=synthesis_input, voice=self.voice, audio_config=self.audio_config
        )

        with open("output.mp3", "wb") as out:
            out.write(self.response.audio_content)

        playsound.playsound('output.mp3', block=False)
        return response




def main(args=None):
    rp.init(args=args)
    tts_service = SpeakTTS()
    rp.spin(tts_service)
    rp.shutdown()


if __name__ == '__main__':
    main()

