% To intensify RGB color images 
function out = intensifyRGB(in, factor)
if(nargin < 2)
    factor = 100;
end
rgbImage = in;
hsv = rgb2hsv(rgbImage);
hChannel = hsv(:, :, 1);
sChannel = hsv(:, :, 2);
vChannel = hsv(:, :, 3);
sChannel = sChannel * factor; 
hsv = cat(3, hChannel, sChannel, vChannel);
out = hsv2rgb(hsv);
end