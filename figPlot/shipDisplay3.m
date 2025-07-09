 function [h1] = shipDisplay3(theta,xShip,yShip,zShip,scale,color)
%SHIPDISPLAY3  Plot a stylized ship icon at a given pose in 2D/3D.
%
%   h1 = shipDisplay3(theta, xShip, yShip, zShip, scale, color)
%   draws a "ship" shape at position (xShip, yShip, zShip), heading theta(1), 
%   and scale. The ship is shown as a 3D patch object with optional color.
%
%   Inputs:
%     theta   - 1¡Á3 vector [yaw, pitch, roll] (rad), ship orientation; 
%               theta(1): heading (z), theta(2): pitch (y), theta(3): roll (x)
%     xShip   - X position of ship center
%     yShip   - Y position of ship center
%     zShip   - Z position (default 0 if empty)
%     scale   - Scalar for ship size (default 1 if empty)
%     color   - 1¡Á3 RGB vector for ship color
%
%   Output:
%     h1      - Handle to the filled patch object representing the ship
%
%   Usage Example:
%     h = shipDisplay3([pi/2 0 0], 10, 10, 0, 1, [0.2 0.6 1]);
%
%   Author: Zhibo He 
%   Date:   2025-04-11

theta=theta*180/pi;
if isempty(zShip)
    zShip=0;
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

    h1=fill3(xy(:,1)+xShip,xy(:,2)+yShip,xy(:,3)+zShip,color);% display the ship picture
    set(h1,'FaceAlpha',1);% Draw a white ship, then overlay another on top  

end