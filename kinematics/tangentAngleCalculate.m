function theta = tangentAngleCalculate(xd)
%TANGENTANGLECALCULATE   Compute the tangent angle (heading) for a 2D curve.
%
%   theta = tangentAngleCalculate(xd)
%   Given a polyline or trajectory matrix xd (N x 3), computes the heading (tangent)
%   angle at each point, expressed in radians.
%
%   Inputs:
%     xd    - [N x 3] array, each row [x, y, psi], where x/y are coordinates and
%             psi is (optional) heading; only x/y are used for computation.
%
%   Outputs:
%     theta - [N x 1] vector, tangent angle (radians) at each trajectory point.
%
%   Notes:
%     - Handles degenerate cases with repeated points at the start.
%     - The last point is assigned the same angle as the penultimate one.
%     - For constant points, returns the provided heading (third column).
%
%   Example:
%     th = tangentAngleCalculate(traj);
%
%   Author: Wenxiang Wu
%   Date:   2025-03-11

    if size(xd,1)>1
        count=1;
        for j=1:size(xd,1)-1
            if xd(j,1)==xd(j+1,1)&&xd(j,2)==xd(j+1,2)
                count=1+count;
            else
                break
            end
        end  
        if count==size(xd,1)
            for j=1:size(xd,1)
                theta(j,1)=xd(j,3);
            end   
        else
            for j=count:size(xd,1)-1
                if xd(j+1,1)-xd(j,1)==0
                    if xd(j+1,2)-xd(j,2)==0
                        theta(j,1)=theta(j-1,1);
                    elseif xd(j+1,2)-xd(j,2)>0
                        theta(j,1)=pi/2;
                    else
                        theta(j,1)=-pi/2;
                    end
                elseif xd(j+1,1)-xd(j,1)>0
                    theta(j,1)=atan((xd(j+1,2)-xd(j,2))/(xd(j+1,1)-xd(j,1)));
                else
                    theta(j,1)=pi+atan((xd(j+1,2)-xd(j,2))/(xd(j+1,1)-xd(j,1)));
                end            
            end                        
            theta(size(xd,1),1)=theta(size(xd,1)-1,1);
            for j=1:count
                theta(j,1)=theta(count,1);
            end   
        end
    else
        theta(1,1)=xd(1,3);
    end   
end

