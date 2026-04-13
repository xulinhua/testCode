import pyaudio
import wave
import os
import struct

class AudioIOManager:
    def __init__(self, input_device_index=24, output_device_index=25):
        """
        初始化音频输入输出管理器
        :param input_device_index: 麦克风设备索引 (hw:2,0 对应索引 24)
        :param output_device_index: 扬声器设备索引 (hw:3,0 对应索引 25)
        """
        self.engine = pyaudio.PyAudio()
        self.input_device_index = input_device_index
        self.output_device_index = output_device_index
        self.input_device_info = self._get_device_info_safely(input_device_index)
        self.output_device_info = self._get_device_info_safely(output_device_index)
        
        # 默认参数
        if self.input_device_info:
            self.default_channels = min(1, int(self.input_device_info['maxInputChannels']))
            # 使用设备的默认采样率而不是硬编码的32000
            self.default_rate = int(self.input_device_info['defaultSampleRate'])
        else:
            self.default_channels = 1
            self.default_rate = 44100  # 默认使用44100Hz
            
        self.default_chunk = 1024
        self.default_format = pyaudio.paInt16
        
        # 音量增益参数 (0.0-2.0, 1.0为原始音量)
        self.volume_gain = 1.0

    def _get_device_info_safely(self, device_index):
        """
        安全地获取设备信息
        :param device_index: 设备索引
        :return: 设备信息字典或None
        """
        try:
            return self.engine.get_device_info_by_index(device_index)
        except Exception as e:
            print(f"获取设备 {device_index} 信息时出错: {e}")
            return None

    def _get_all_supported_sample_rates(self):
        """
        获取设备支持的所有采样率
        :return: 支持的采样率列表
        """
        # 根据项目经验，在Jetson平台上直接检测USB设备采样率可能不可靠
        # 返回一组常用的采样率，让播放函数尝试哪个能工作
        return [48000, 44100, 32000, 96000]
        
    def _get_supported_sample_rate(self):
        """
        获取设备支持的采样率
        :return: 支持的采样率列表中的第一个（默认选择）
        """
        # 直接返回已知的设备支持采样率，避免ALSA错误
        return 32000

    def set_volume_gain(self, gain):
        """
        设置音量增益
        :param gain: 音量增益系数 (0.0-2.0)
        """
        self.volume_gain = max(0.0, min(2.0, gain))

    def capture_audio_stream(self, chunk=None, channels=None, rate=None, format=None, callback=None):
        """
        实时获取麦克风设备的音频数据
        :param chunk: 音频块大小
        :param channels: 声道数
        :param rate: 采样率
        :param format: 音频格式
        :param callback: 回调函数，用于处理实时音频数据
        :return: 音频流对象
        """
        chunk = chunk or self.default_chunk
        channels = channels or self.default_channels
        rate = rate or self.default_rate
        format = format or self.default_format
        
        # 确保声道数不超过设备支持的最大声道数
        if self.input_device_info:
            max_input_channels = int(self.input_device_info['maxInputChannels'])
            channels = min(channels, max_input_channels)
        
        # 检查输入设备是否支持指定的采样率
        try:
            if self.input_device_info and not self.engine.is_format_supported(
                rate=rate,
                input_device=self.input_device_index,
                input_channels=channels,
                input_format=format,
                output_device=None,
                output_channels=None,
                output_format=None
            ):
                print(f"警告: 输入设备不支持采样率 {rate} Hz")
        except Exception as e:
            print(f"检查输入设备采样率时出错: {e}")
        
        stream = self.engine.open(
            format=format,
            channels=channels,
            rate=rate,
            input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=chunk
        )
        
        device_name = self.input_device_info['name'] if self.input_device_info else f"设备 {self.input_device_index}"
        print(f"开始从设备 {device_name} 实时采集音频...")
        
        try:
            while True:
                # 添加exception_on_overflow=False参数来避免溢出错误
                data = stream.read(chunk, exception_on_overflow=False)
                # 应用音量增益
                if self.volume_gain != 1.0:
                    # 手动实现音量增益而不依赖numpy
                    audio_data = bytearray(data)
                    for i in range(0, len(audio_data), 2):
                        # 将两个字节组合成16位有符号整数
                        sample = int.from_bytes(audio_data[i:i+2], byteorder='little', signed=True)
                        # 应用增益并限制范围
                        sample = int(sample * self.volume_gain)
                        sample = max(-32768, min(32767, sample))
                        # 转换回字节
                        sample_bytes = sample.to_bytes(2, byteorder='little', signed=True)
                        audio_data[i] = sample_bytes[0]
                        audio_data[i+1] = sample_bytes[1]
                    data = bytes(audio_data)
                
                if callback:
                    callback(data)
        except KeyboardInterrupt:
            print("停止音频采集")
        finally:
            stream.stop_stream()
            stream.close()
            
        return stream

    def capture_and_save_audio(self, filename, record_seconds=5, chunk=None, channels=None, rate=None, format=None):
        """
        获取麦克风设备的音频数据并保存到文件
        :param filename: 保存的文件名
        :param record_seconds: 录制时长(秒)
        :param chunk: 音频块大小
        :param channels: 声道数
        :param rate: 采样率
        :param format: 音频格式
        """
        chunk = chunk or self.default_chunk
        channels = channels or self.default_channels
        rate = rate or self.default_rate
        format = format or self.default_format
        
        # 确保声道数不超过设备支持的最大声道数
        if self.input_device_info:
            max_input_channels = int(self.input_device_info['maxInputChannels'])
            channels = min(channels, max_input_channels)
        
        # 检查输入设备是否支持指定的采样率
        try:
            if self.input_device_info and not self.engine.is_format_supported(
                rate=rate,
                input_device=self.input_device_index,
                input_channels=channels,
                input_format=format,
                output_device=None,
                output_channels=None,
                output_format=None
            ):
                print(f"警告: 输入设备不支持采样率 {rate} Hz")
        except Exception as e:
            print(f"检查输入设备采样率时出错: {e}")
        
        # 确保output目录存在
        output_dir = os.path.join(os.path.dirname(__file__), 'output')
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        filepath = os.path.join(output_dir, filename)
        
        # 打开音频流
        stream = self.engine.open(
            format=format,
            channels=channels,
            rate=rate,
            input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=chunk
        )
        
        print(f"开始录制 {record_seconds} 秒音频到 {filepath}")
        
        frames = []
        for i in range(0, int(rate / chunk * record_seconds)):
            # 添加overflow_exception=False参数来避免溢出错误
            data = stream.read(chunk, exception_on_overflow=False)
            # 应用音量增益
            if self.volume_gain != 1.0:
                # 手动实现音量增益而不依赖numpy
                audio_data = bytearray(data)
                for j in range(0, len(audio_data), 2):
                    # 将两个字节组合成16位有符号整数
                    sample = int.from_bytes(audio_data[j:j+2], byteorder='little', signed=True)
                    # 应用增益并限制范围
                    sample = int(sample * self.volume_gain)
                    sample = max(-32768, min(32767, sample))
                    # 转换回字节
                    sample_bytes = sample.to_bytes(2, byteorder='little', signed=True)
                    audio_data[j] = sample_bytes[0]
                    audio_data[j+1] = sample_bytes[1]
                data = bytes(audio_data)
            frames.append(data)
        
        stream.stop_stream()
        stream.close()
        
        # 保存为WAV文件
        wf = wave.open(filepath, 'wb')
        wf.setnchannels(channels)
        wf.setsampwidth(self.engine.get_sample_size(format))
        wf.setframerate(rate)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        print(f"音频已保存到 {filepath}")
        return filepath

    def _resample_audio_data(self, audio_data, original_rate, target_rate, sample_width=2):
        """
        对音频数据进行重采样
        :param audio_data: 原始音频数据(bytes)
        :param original_rate: 原始采样率
        :param target_rate: 目标采样率
        :param sample_width: 采样宽度(字节数)，1=8位，2=16位
        :return: 重采样后的音频数据(bytes)
        """
        if original_rate == target_rate:
            return audio_data
            
        # 如果没有数据，直接返回
        if len(audio_data) == 0:
            return audio_data
            
        # 计算重采样因子
        resample_factor = float(target_rate) / float(original_rate)
        
        # 根据采样宽度确定参数
        sample_size = sample_width
        num_samples = len(audio_data) // sample_size
        
        # 如果采样点数太少，直接返回原始数据以避免问题
        if num_samples < 2:
            return audio_data
            
        # 计算新采样点数量
        new_num_samples = int(num_samples * resample_factor)
        
        # 如果目标长度为0，返回空数据
        if new_num_samples == 0:
            return b''
        
        # 将字节数据转换为采样点列表
        samples = []
        if sample_width == 1:
            # 8位音频数据(无符号)
            for i in range(0, len(audio_data), sample_size):
                # 8位音频是无符号的(0-255)
                sample = audio_data[i]
                # 转换为有符号范围(-128到127)以便处理
                sample = sample - 128
                samples.append(sample)
        else:
            # 16位或更高位深度音频数据(有符号)
            for i in range(0, len(audio_data), sample_size):
                # 将字节组合成有符号整数
                sample = int.from_bytes(audio_data[i:i+sample_size], byteorder='little', signed=True)
                samples.append(sample)
        
        # 使用线性插值重采样
        resampled = bytearray()
        for i in range(new_num_samples):
            # 计算在原始数据中的位置
            pos = i / resample_factor
            left_index = int(pos)
            
            # 边界检查
            if left_index >= len(samples):
                left_index = len(samples) - 1
            if left_index < 0:
                left_index = 0
                
            right_index = min(left_index + 1, len(samples) - 1)
            
            # 线性插值
            if left_index == right_index:
                # 如果左右索引相同，直接使用该点的值
                sample_value = samples[left_index]
            else:
                # 线性插值
                fraction = pos - left_index
                sample_value = int(samples[left_index] * (1.0 - fraction) + samples[right_index] * fraction)
            
            # 根据采样宽度转换为字节
            if sample_width == 1:
                # 8位音频数据(无符号)
                # 转换回无符号范围(0-255)
                sample_value = sample_value + 128
                sample_value = max(0, min(255, sample_value))
                resampled.append(sample_value)
            else:
                # 16位或更高位深度音频数据(有符号)
                sample_value = max(-32768, min(32767, sample_value))
                sample_bytes = sample_value.to_bytes(sample_size, byteorder='little', signed=True)
                resampled.extend(sample_bytes)
        
        return bytes(resampled)

    def play_audio_file(self, filepath=None, chunk=None, channels=None, rate=None, format=None):
        """
        播放音频文件
        :param filepath: 音频文件路径，默认为项目目录下的1.wav
        :param chunk: 音频块大小
        :param channels: 声道数
        :param rate: 采样率
        :param format: 音频格式
        """
        if filepath is None:
            filepath = os.path.join(os.path.dirname(__file__), '1.wav')
            
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"音频文件 {filepath} 不存在")
            
        chunk = chunk or self.default_chunk
        channels = channels or self.default_channels
        format = format or self.default_format
        
        # 打开音频文件
        wf = wave.open(filepath, 'rb')
        
        # 使用音频文件的采样率
        file_rate = wf.getframerate()
        file_channels = wf.getnchannels()
        file_sampwidth = wf.getsampwidth()  # 获取采样宽度
        
        # 根据采样宽度确定PyAudio格式
        if file_sampwidth == 1:
            file_format = pyaudio.paUInt8  # 8位音频
        elif file_sampwidth == 2:
            file_format = pyaudio.paInt16  # 16位音频
        else:
            file_format = self.default_format  # 使用默认格式
            
        print(f"音频文件信息: 采样率={file_rate}Hz, 声道数={file_channels}, 采样宽度={file_sampwidth}字节")
        
        # 获取设备支持的采样率
        supported_rates = self._get_all_supported_sample_rates()
        print(f"设备支持的采样率: {supported_rates}")
        
        # 优先选择与文件采样率相同的采样率（如果支持）
        target_rate = file_rate
        if target_rate not in supported_rates:
            # 如果文件采样率不支持，选择最接近的支持采样率
            target_rate = min(supported_rates, key=lambda x: abs(x - file_rate))
        print(f"将使用采样率: {target_rate} Hz")
        
        # 尝试打开输出流，如果失败则尝试其他支持的采样率
        stream = None
        # 首先尝试文件采样率或最接近的采样率
        try_rates = [target_rate] + [r for r in supported_rates if r != target_rate]
        for try_rate in try_rates:
            try:
                stream = self.engine.open(
                    format=file_format,
                    channels=file_channels,
                    rate=try_rate,
                    output=True,
                    output_device_index=self.output_device_index,
                    frames_per_buffer=chunk
                )
                target_rate = try_rate
                print(f"成功使用采样率: {target_rate} Hz")
                break
            except Exception as e:
                print(f"尝试采样率 {try_rate} Hz 失败: {e}")
                continue
                
        if stream is None:
            wf.close()
            raise RuntimeError("无法使用任何支持的采样率打开音频流")
        
        device_name = self.output_device_info['name'] if self.output_device_info else f"设备 {self.output_device_index}"
        print(f"开始播放 {filepath} 到设备 {device_name}")
        
        # 读取并播放音频数据
        data = wf.readframes(chunk)
        sample_count = 0
        while data:
            # 如果需要进行采样率转换
            if target_rate != file_rate:
                # 将数据进行重采样
                resampled_data = self._resample_audio_data(data, file_rate, target_rate, file_sampwidth)
                data_to_play = resampled_data
                sample_count += len(resampled_data) // file_sampwidth
            else:
                data_to_play = data
                sample_count += len(data) // file_sampwidth
                
            # 应用音量增益（仅对16位音频）
            if self.volume_gain != 1.0 and file_sampwidth == 2:
                # 手动实现音量增益而不依赖numpy
                audio_data = bytearray(data_to_play)
                for j in range(0, len(audio_data), 2):
                    # 将两个字节组合成16位有符号整数
                    sample = int.from_bytes(audio_data[j:j+2], byteorder='little', signed=True)
                    # 应用增益并限制范围
                    sample = int(sample * self.volume_gain)
                    sample = max(-32768, min(32767, sample))
                    # 转换回字节
                    sample_bytes = sample.to_bytes(2, byteorder='little', signed=True)
                    audio_data[j] = sample_bytes[0]
                    audio_data[j+1] = sample_bytes[1]
                data_to_play = bytes(audio_data)
            elif self.volume_gain != 1.0 and file_sampwidth == 1:
                # 对8位音频应用音量增益
                audio_data = bytearray(data_to_play)
                for j in range(len(audio_data)):
                    # 8位音频是无符号的(0-255)
                    sample = audio_data[j]
                    # 应用增益并限制范围
                    sample = int(sample * self.volume_gain)
                    sample = max(0, min(255, sample))
                    audio_data[j] = sample
                data_to_play = bytes(audio_data)
                
            stream.write(data_to_play)
            data = wf.readframes(chunk)
        
        # 关闭流
        stream.stop_stream()
        stream.close()
        wf.close()
        
        print("播放完成")

    def play_audio_data(self, audio_data, chunk=None, channels=None, rate=None, format=None):
        """
        播放音频数据
        :param audio_data: 音频数据 (bytes)
        :param chunk: 音频块大小
        :param channels: 声道数
        :param rate: 采样率
        :param format: 音频格式
        """
        chunk = chunk or self.default_chunk
        channels = channels or self.default_channels
        rate = rate or self.default_rate
        format = format or self.default_format
        
        # 不再使用is_format_supported检查，直接尝试打开流
        # 因为is_format_supported在某些ALSA配置下可能返回错误结果
        
        # 打开输出流
        stream = self.engine.open(
            format=format,
            channels=channels,
            rate=rate,
            output=True,
            output_device_index=self.output_device_index,
            frames_per_buffer=chunk
        )
        
        device_name = self.output_device_info['name'] if self.output_device_info else f"设备 {self.output_device_index}"
        print(f"开始播放音频数据到设备 {device_name}")
        
        # 播放音频数据
        # 应用音量增益
        data = audio_data
        if self.volume_gain != 1.0:
            # 手动实现音量增益而不依赖numpy
            # 假设是16位音频数据(2字节每个采样点)
            sample_width = 2
            audio_data_array = bytearray(data)
            for j in range(0, len(audio_data_array), sample_width):
                # 将两个字节组合成16位有符号整数
                sample = int.from_bytes(audio_data_array[j:j+sample_width], byteorder='little', signed=True)
                # 应用增益并限制范围
                sample = int(sample * self.volume_gain)
                sample = max(-32768, min(32767, sample))
                # 转换回字节
                sample_bytes = sample.to_bytes(sample_width, byteorder='little', signed=True)
                audio_data_array[j] = sample_bytes[0]
                audio_data_array[j+1] = sample_bytes[1]
            data = bytes(audio_data_array)
            
        stream.write(data)
        
        # 关闭流
        stream.stop_stream()
        stream.close()
        
        print("播放完成")

    def close(self):
        """关闭音频引擎"""
        self.engine.terminate()