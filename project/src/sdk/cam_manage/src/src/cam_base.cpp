#include "../include/cam_manage/cam_base.hpp"
#include <iostream>

//std::vector<CamDevInfo> CamBase::cam_info_list_;
RtnType CamBase::get_exposure_range(float &min_exposure, float &max_exposure, CamStreamType modu)
{
    return get_cam_para_range(CamParaType::PARA_EXPOSURE, min_exposure, max_exposure, modu);
}
RtnType CamBase::get_exposure_val(float &exposure, CamStreamType modu)
{
    return get_cam_para(CamParaType::PARA_EXPOSURE, exposure, modu);
}
RtnType CamBase::set_exposure_val(float exposure, CamStreamType modu)
{
    float min_val, max_val;
    RtnType rtn = get_exposure_range(min_val, max_val, modu);
    if (rtn == RtnType::RTN_SUCCESS)
    {
        if (exposure < min_val)
            exposure = min_val;
        if (exposure > max_val)
            exposure = max_val;
        return set_cam_para(CamParaType::PARA_EXPOSURE, exposure, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE;
    }
}

// 帧率
RtnType CamBase::get_acqu_fps_enable(bool &enable, CamStreamType modu)
{
    return get_cam_para_enable(CamParaType::PARA_FPS, modu, enable);
}
RtnType CamBase::set_acqu_fps_enable(bool enable, CamStreamType modu)
{
    return set_cam_para_enable(CamParaType::PARA_FPS, modu, enable);
}
RtnType CamBase::get_fps_range(int &min_fps, int &max_fps, CamStreamType modu)
{
    return get_cam_para_range(CamParaType::PARA_FPS, min_fps, max_fps, modu);
}
RtnType CamBase::get_fps(int &fps, CamStreamType modu)
{
    return get_cam_para(CamParaType::PARA_FPS, fps, modu);
}
RtnType CamBase::set_fps(int fps, CamStreamType modu)
{
    int min_val, max_val;
    RtnType rtn = get_fps_range(min_val, max_val, modu);
    if (rtn == RtnType::RTN_SUCCESS)
    {
        if (fps < min_val)
            fps = min_val;
        if (fps > max_val)
            fps = max_val;
        return set_cam_para(CamParaType::PARA_FPS, fps, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE;
    }
}

// 增益
RtnType CamBase::get_gain_range(float &min_gain, float &max_gain, CamStreamType modu)
{
    return get_cam_para_range(CamParaType::PARA_GAIN, min_gain, max_gain, modu);
}
RtnType CamBase::get_gain_val(float &gain, CamStreamType modu)
{
    return get_cam_para(CamParaType::PARA_GAIN, gain, modu);
}
RtnType CamBase::set_gain_val(float gain, CamStreamType modu)
{
    float min_val, max_val;
    RtnType rtn = get_gain_range(min_val, max_val, modu);
    if (rtn == RtnType::RTN_SUCCESS)
    {
        if (gain < min_val)
            gain = min_val;
        if (gain > max_val)
            gain = max_val;
        return set_cam_para(CamParaType::PARA_GAIN, gain, modu);
    }
    else
    {
        return RtnType::RTN_FAILIURE;
    }
}

// 亮度
RtnType CamBase::get_brightness_range(float &min_bright, float &max_bright)
{
    return get_cam_para_range(CamParaType::PARA_BRIGHTNESS, min_bright, max_bright, CamStreamType::STREAM_COLOR);
}
RtnType CamBase::get_brightness(float &bright)
{
    return get_cam_para(CamParaType::PARA_BRIGHTNESS, bright, CamStreamType::STREAM_COLOR);
}
RtnType CamBase::set_brightness(float bright)
{
    float min_val, max_val;
    RtnType rtn = get_brightness_range(min_val, max_val);
    if (rtn == RtnType::RTN_SUCCESS)
    {
        if (bright < min_val)
            bright = min_val;
        if (bright > max_val)
            bright = max_val;
        return set_cam_para(CamParaType::PARA_BRIGHTNESS, bright, CamStreamType::STREAM_COLOR);
    }
    else
    {
        return RtnType::RTN_FAILIURE;
    }
}

// Gamma
RtnType CamBase::get_gamma_range(float &min_gamma, float &max_gamma)
{
    return get_cam_para_range(CamParaType::PARA_GAMMA, min_gamma, max_gamma, CamStreamType::STREAM_COLOR);
}
RtnType CamBase::get_gamma(float &gamma)
{
    return get_cam_para(CamParaType::PARA_GAMMA, gamma, CamStreamType::STREAM_COLOR);
}
RtnType CamBase::set_gamma(float gamma)
{
    float min_val, max_val;
    RtnType rtn = get_gamma_range(min_val, max_val);
    if (rtn == RtnType::RTN_SUCCESS)
    {
        if (gamma < min_val)
            gamma = min_val;
        if (gamma > max_val)
            gamma = max_val;
        return set_cam_para(CamParaType::PARA_GAMMA, gamma, CamStreamType::STREAM_COLOR);
    }
    else
    {
        return RtnType::RTN_FAILIURE;
    }
}