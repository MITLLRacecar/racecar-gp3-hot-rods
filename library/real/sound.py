import sounddevice as sd
import soundfile as sf
import numpy as np


class Sound:

    __DEFAULT_SPEAKER = "USB2.0 Device"
    __DEFAULT_MIC = "USB PnP Sound Device"

    def __init__(self, speaker=__DEFAULT_SPEAKER, mic=__DEFAULT_MIC):

        self.__speaker = None
        self.__mic = None
        self.__speaker_sample_rate = None
        self.__mic_sample_rate = None
        self.__output_stream = None
        self.__input_stream = None
        self.__file = None
        self.__play = False
        self.__rec = False
        self.__rec_time = 0

        self.set_speaker(speaker)
        self.set_mic(mic)
        self.set_output_stream()
        self.set_input_stream()

    def set_speaker(self, device):
        """
        Updates the speaker to be the given device.

        Args:
            device: (string or int) A substring of the device name or an
                identifying ID number.

        Note:
            If you choose to input a substring of the device name, it
            must be unique to all other devices. If the substring is
            found in multiple device names, the update will fail. You can
            find information about available devices using list_devices.

            Updating the speaker does not update the output stream. The
            stream must be updated separately.

        Example:
            sound = Sound()
            sound.set_speaker('USB2.0 Device')
            sound.set_output_stream()
        """
        try:
            assert isinstance(device, str) or isinstance(
                device, int
            ), "device must be an ID (int) or substring of device name"
            speaker = sd.query_devices(device=device)
            self.__speaker = speaker["name"]
            self.__speaker_sample_rate = int(speaker["default_samplerate"])
        except ValueError as e:
            print("Could not set speaker: ")
            print(e)
        except AssertionError as e:
            print(e)

    def set_mic(self, device):
        """
        Updates the mic to be the given device.

        Args:
            device: (string or int) A unique substring of the device name or an
                identifying ID number.
        Note:
            If you choose to input a substring of the device name, it
            must be unique to all other devices. If the substring is
            found in multiple device names, the update will fail. You can
            find information about available devices using list_devices.

            Updating the mic does not update the input stream. The
            stream must be updated separately.

        Example:
            sound = Sound()
            sound.set_mic('USB PnP')
            sound.set_input_stream()
        """
        try:
            assert isinstance(device, str) or isinstance(
                device, int
            ), "device must be an ID (int) or substring of device name"
            mic = sd.query_devices(device=device)
            self.__mic = mic["name"]
            self.__mic_sample_rate = int(mic["default_samplerate"])
        except ValueError as e:
            print("Could not set mic: ")
            print(e)

    def set_output_stream(self):
        """
        Opens an output stream with the currently set speaker.

        Note:
            The output stream retrieves the speaker device from self.__speaker,
            which can be set using set_speaker.

        Example:
            sound = Sound()
            sound.set_speaker('USB2.0 Device')
            sound.set_output_stream()
        """
        try:
            assert isinstance(
                self.__speaker, unicode
            ), "Speaker could not be found. Speaker: " + str(self.__speaker)
            self.__output_stream = sd.OutputStream(
                samplerate=self.__speaker_sample_rate,
                device=self.__speaker,
                channels=1,
                dtype="float32",
            )
        except AssertionError as e:
            print(e)

    def set_input_stream(self):
        """
        Opens an output stream with the currently set mic.

        Note:
            The output stream retrieves the mic device from self.__mic,
            which can be set using set_mic.

        Example:
            sound = Sound()
            sound.set_mic('USB PnP')
            sound.set_input_stream()
        """
        try:
            print(type(self.__mic))
            assert isinstance(
                self.__mic, unicode
            ), "Mic could not be found. Mic: " + str(self.__mic)
            self.__input_stream = sd.InputStream(
                samplerate=self.__mic_sample_rate,
                device=self.__mic,
                channels=1,
                dtype="float32",
            )
        except AssertionError as e:
            print(e)

    def play_audio(self, filename):
        """
        Hangs the running process to play audio from specified file.

        Args:
            filename: (string) The full filename of the audio file you
                want to play

        Note:
            Processes that call this function must wait until the full audio
            file is done playing and the function returns. Additionally,
            the given file must be in the SAME folder as the script calling
            the function.

        Example:
            sound = Sound()
            sound.play_audio('guitar.wav')
        """
        try:
            data, sample_rate = sf.read(filename, dtype="float32")
            self.__output_stream.start()
            self.__output_stream.write(data)
            self.__output_stream.stop()
        except AttributeError as e:
            print(
                "Output stream could not start. Output stream: "
                + str(type(self.__output_stream))
            )

    def record_audio(self, filename, seconds):
        """
        Hangs the running process to a record an audio file to the specified file.

        Args:
            filename: (string) The full filename of the audio file you
                want to record to
            seconds: (int) The number of seconds you would like to record audio

        Note:
            Processes that call this function must wait until the full audio
            file is done recording and the function returns. Additionally,
            the given file will appear in the SAME folder as the script calling
            the function. If a file of the same name already exists, it will
            be overwritten.

        Example:
            sound = Sound()
            sound.record_audio('guitar.wav')
            sound.play_audio('guitar.wav')

        """
        try:
            self.__input_stream.start()
            frames = int(seconds * self.__mic_sample_rate)
            if self.__speaker_sample_rate == None:
                self.__speaker_sample_rate = self.__mic_sample_rate
            soundfile = sf.SoundFile(
                file=filename,
                mode="w",
                samplerate=self.__speaker_sample_rate,
                channels=1,
            )
            data = self.__input_stream.read(frames)[0]
            self.__input_stream.stop()
            soundfile.write(data)
        except AttributeError as e:
            print(
                "Input stream could not start. Input stream: "
                + str(type(self.__input_stream))
            )

    def play(self, filename):
        """
        Acts as a signal for sound thread to play audio.

        Args:
            filename: (string) The full filename of the audio file you
                want to play

        Note:
            Play is meant to act as a thread signal if any threading is
            implemented. It updates the self.__file variable and self.__play.
        """
        try:
            assert isinstance(
                filename, str
            ), "Invalid filename. Filename must be type str."
            self.__file = filename
            self.__play = True
        except AssertionError as e:
            print(e)

    def rec(self, filename, seconds):
        """
        Acts as a signal for sound thread to record audio.

        Args:
            filename: (string) The full filename of the audio file you.
                want to play
            seconds: (int) The number of seconds you would like to record audio

        Note:
            Rec is meant to act as a thread signal if any threading is
            implemented. It updates the self.__file variable and self.__rec.
        """
        try:
            assert isinstance(
                filename, str
            ), "Invalid filename. Filename must be type str."
            self.__file = filename
            self.__rec = True
            self.__rec_time = seconds
        except AssertionError as e:
            print(e)

    def __play_file(self):
        """
        Hangs the running process to play audio from specified file.

        Note:
            When called directly, a process must hang until the audio finishes
            playing. If called by a thread, the function avoids hanging by
            breaking up the file into smaller chunks. When self.__play or
            self__rec change, the stream stops.

        Example:
            if self.__play:
                self.__play = False
                self.__play_file()
        """
        data, sample_rate = sf.read(file=self.__file, dtype="float32")
        n, d = np.shape(data)
        outdata = []
        self.__output_stream.start()
        while not (self.__play) and not (self.__rec) and (n != 0):
            frames = self.__output_stream.write_available
            outdata = data[:frames]
            data = data[frames:]
            self.__output_stream.write(outdata)
            n, d = np.shape(data)
        self.__output_stream.stop()

    def __rec_file(self):
        """
        Hangs the running process to record audio to a specified file.

        Note:
            When called directly, the process must hang until the audio finishes
            recording. If called by a thread, the function avoids hanging by
            breaking up the recording into smaller chunks. When self.__play or
            self__rec change, the stream stops.

            Breaks up the reading into chunks to allow interrupt and avoid wait.

        Example:
            if self.__rec:
                self.__rec = False
                self.__rec_file()
        """
        total_frames = int(self.__rec_time * self.__mic_sample_rate)
        soundfile = sf.SoundFile(
            file=self.__file, mode="w", samplerate=self.__mic_sample_rate, channels=1
        )
        indata = None
        self.__input_stream.start()
        while not (self.__play) and not (self._rec) and total_frames > 0:
            frames = self.__output_stream.read_available()
            indata = self.__output_stream.read(frames)
            soundfile.write(indata)
            total_frames -= frames
        self.__input_stream.stop()

    def set_file(self, filename):
        """
        Updates the file to be the given filename.

        Args:
            filename: (str) The full filename of the audio file you
                want to record to.

        Note:
            This function allows the user to manually set the file that is
            played from or recorded to. It is intended for functions which
            do not have a filename argument, such as __play_file and __rec_file.

        Example:
            sound = Sound()
            sound.set_file('guitar.wav')
            sould.__play_file()
        """
        if isinstance(filename, str):
            self.__file = filename
        else:
            print(
                "Filename must be of type str. Filename: "
                + str(filename)
                + str(type(filename))
            )

    def list_devices(self):
        """
        Lists all available devices.

        Note:
            The resulting list of devices feature identifying integer IDs
            and device names. You can use one of these to update the speaker
            and mic to non default options.

        Example:
            sound = Sound()
            sound.list_devices()
            sound.set_mic('USB PnP')
        """
        devices = sd.query_devices()
        print("Your available devices:")
        print(devices)
