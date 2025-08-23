 function [h1] = ASVDisplay3(theta,xASV,yASV,zASV,scale,color)
%ASVDISPLAY3  Plot a stylized ASV icon at a given pose in 2D/3D.
%
%   h1 = ASVDisplay3(theta, xASV, yASV, zASV, scale, color)
%   draws a "ASV" shape at position (xASV, yASV, zASV), heading theta(1), 
%   and scale. The ASV is shown as a 3D patch object with optional color.
%
%   Inputs:
%     theta   - 1¡Á3 vector [yaw, pitch, roll] (rad), ASV orientation; 
%               theta(1): heading (z), theta(2): pitch (y), theta(3): roll (x)
%     xASV   - X position of ASV center
%     yASV   - Y position of ASV center
%     zASV   - Z position (default 0 if empty)
%     scale   - Scalar for ASV size (default 1 if empty)
%     color   - 1¡Á3 RGB vector for ASV color
%
%   Output:
%     h1      - Handle to the filled patch object representing the ASV
%
%   Usage Example:
%     h = ASVDisplay3([pi/2 0 0], 10, 10, 0, 1, [0.2 0.6 1]);
%
%   Author: Zhibo He 
%   Date:   2025-04-11

theta=theta*180/pi;
if isempty(zASV)
    zASV=0;
end
if isempty(scale)
    scale=1;
end
theta(1) =90-theta(1);
    %%top
    x           = (-0.5:0.01:0.5)*scale;
    y           = 0.125*sin(4*pi*x/scale)*scale.*(x>=-0.5*scale&x<-0.375*scale)...
                  +0.125*scale.*(x>=-0.375*scale&x<0.125*scale)...
                  +0.125*scale*cos(4*pi*x/3/scale-pi/6).*(x>=0.125*scale&x<=0.5*scale);
    x           = x';
    y2          = -y;
    y2          = y2';
    y           = y';
    tmpxy       = [x y];
    tmpxy1      = [x y2];
    tmpxy1      = flipud(tmpxy1);
    matrot_z    = [cosd(-theta(1)),-sind(-theta(1)),0;...
                   sind(-theta(1)),cosd(-theta(1)),0;
                   0,0,1];
    matrot_y    = [cosd(-theta(2)),0,sind(-theta(2));...
                   0,1,0;
                   -sind(-theta(2)),0,cosd(-theta(2))];
    matrot_x    = [1,0,0;...
                   0,cosd(-theta(3)),-sind(-theta(3));
                   0,sind(-theta(3)),cosd(-theta(3))];
    xy          = [tmpxy;tmpxy1];
        z           = ones(length(xy),1)*scale*0.1;
    xy          = [xy,z];
    xy          = xy*matrot_y*matrot_x*matrot_z;

    h1=fill3(xy(:,1)+xASV,xy(:,2)+yASV,xy(:,3)+zASV,color);% display the ASV picture
    set(h1,'FaceAlpha',1);% Draw a white ASV, then overlay another on top  

end
