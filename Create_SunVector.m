clc
clear all

epoch = [2011 08 16 17 00 00];

for t = 1:24*60*60*10
    [R_SUN_ECI, DCM_ECItoLVLH]   = SunVector(epoch,t);
    SunVec_LVLH(:,t) = DCM_ECItoLVLH*R_SUN_ECI;
end

