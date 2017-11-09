% dubinsParameters
%   - Find Dubin's parameters between two configurations
%
% input is:
%   start_node  - [wn_s, we_s, wd_s, chi_s, 0, 0]
%   end_node    - [wn_e, wn_e, wd_e, chi_e, 0, 0]
%   R           - minimum turn radius
%
% output is:
%   dubinspath  - a matlab structure with the following fields
%       dubinspath.ps   - the start position in re^3
%       dubinspath.chis - the start course angle
%       dubinspath.pe   - the end position in re^3
%       dubinspath.chie - the end course angle
%       dubinspath.R    - turn radius
%       dubinspath.L    - length of the Dubins path
%       dubinspath.cs   - center of the start circle
%       dubinspath.lams - direction of the start circle
%       dubinspath.ce   - center of the end circle
%       dubinspath.lame - direction of the end circle
%       dubinspath.w1   - vector in re^3 defining half plane H1
%       dubinspath.q1   - unit vector in re^3 along straight line path
%       dubinspath.w2   - vector in re^3 defining position of half plane H2
%       dubinspath.w3   - vector in re^3 defining position of half plane H3
%       dubinspath.q3   - unit vector defining direction of half plane H3
% 

function dubinspath = dubinsParameters(start_node, end_node, R)

  ell = norm(start_node(1:2)-end_node(1:2));
  dubinspath = [];
  if ell<2*R-1e-6,
      disp('The distance between nodes must be larger than 2R.');
%       dubinspath = [];
  else
    Rz = @(x) [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1];
    ang = @(x) mod(x,2*pi);
    NORM = @(x) x/norm(x);
    vec_angle = @(s,e) atan2(e(2)-s(2),e(1)-s(1));
      
    ps   = start_node(1:3)';
    chis = start_node(4);
    pe   = end_node(1:3)';
    chie = end_node(4);
    
    crs = ps + R*Rz(pi/2)*[cos(chis);sin(chis);0];
    cls = ps + R*Rz(-pi/2)*[cos(chis);sin(chis);0];
    cre = pe + R*Rz(pi/2)*[cos(chie);sin(chie);0];
    cle = pe + R*Rz(-pi/2)*[cos(chie);sin(chie);0];

  % compute L1
    theta = vec_angle(crs,cre);
    L1 = norm(crs-cre) + R*ang(2*pi+ang(theta-pi/2)-ang(chis-pi/2))...
        +R*ang(2*pi+ang(chie-pi/2)-ang(theta-pi/2));
    
    % compute L2
    theta = vec_angle(crs,cle);
    theta2 = theta-pi/2+asin(2*R/ell);
    if isreal(theta2)==0
      L2 = 9999; 
    else
      L2 = sqrt(ell^2-4*R^2) + R*ang(2*pi+ang(theta2)-ang(chis-pi/2))...
          +R*ang(2*pi+ang(theta2+pi)-ang(chie+pi/2));
    end
    % compute L3
    theta = vec_angle(cls,cre);
    theta2 = acos(2*R/ell);
    if isreal(theta2)==0
      L3 = 9999;
    else
      L3 = sqrt(ell^2-4*R^2) + R*ang(2*pi+ang(chis+pi/2)-ang(theta+theta2))...
          +R*ang(2*pi+ang(chie-pi/2)-ang(theta+theta2-pi));
    end
    % compute L4
    theta = vec_angle(cls,cle);
    L4 = norm(cls-cle)+R*ang(2*pi+ang(chis+pi/2)-ang(theta+pi/2))...
        +R*ang(2*pi+ang(theta+pi/2)-ang(chie+pi/2));
    
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    %disp(['Case ' num2str(idx)])
    e1 = [1; 0; 0];
    switch(idx)
        case 1
            cs = crs;
            lams = 1;
            ce = cre;
            lame = 1;
            q1 = NORM(ce-cs);
            w1 = cs+R*Rz(-pi/2)*q1;
            w2 = ce+R*Rz(-pi/2)*q1;
        case 2   
            cs = crs;
            lams = 1;
            ce = cle;
            lame = -1;
            theta = vec_angle(cs,ce);
            ell = norm(ce-cs);
            if ell < 2*R
                %disp('Circle centers too close')
                return;
            end
            theta2 = theta-pi/2+asin(2*R/ell);
            q1 = Rz(theta2+pi/2)*e1;
            w1 = cs+R*Rz(theta2)*e1;
            w2 = ce+R*Rz(theta2+pi)*e1;
        case 3
            cs = cls;
            lams = -1;
            ce = cre;
            lame = 1;
            theta = vec_angle(cs,ce);
            ell = norm(ce-cs);
            if ell < 2*R
                %disp('Circle centers too close')
                return;
            end
            theta2 = acos(2*R/ell);
            q1 = Rz(theta+theta2-pi/2)*e1;
            w1 = cs+R*Rz(theta+theta2)*e1;
            w2 = ce+R*Rz(theta+theta2-pi)*e1;
         case 4
            cs = cls;
            lams = -1;
            ce = cle;
            lame = -1;
            q1 = NORM(ce-cs);
            w1 = cs+R*Rz(pi/2)*q1;
            w2 = ce+R*Rz(pi/2)*q1;
    end
    w3 = pe;
    q3 = Rz(chie)*e1;
    
    % assign path variables
    dubinspath.ps   = ps;
    dubinspath.chis = chis;
    dubinspath.pe   = pe;
    dubinspath.chie = chie;
    dubinspath.R    = R;
    dubinspath.L    = L;
    dubinspath.cs   = cs;
    dubinspath.lams = lams;
    dubinspath.ce   = ce;
    dubinspath.lame = lame;
    dubinspath.w1   = w1;
    dubinspath.q1   = q1;
    dubinspath.w2   = w2;
    dubinspath.w3   = w3;
    dubinspath.q3   = q3;
  end
end
   
    
%     crs = ps + R*rotz(pi/2)*[cos(chis),sin(chis),0]';
%     cls = ps + R*rotz(-pi/2)*[cos(chis),sin(chis),0]';
%     cre = pe + R*rotz(pi/2)*[cos(chie),sin(chie),0]';
%     cle = pe + R*rotz(-pi/2)*[cos(chie),sin(chie),0]';
    
   
%     % compute L1
%     
%     theta = mo(atan2(cre(2)-crs(2), cre(1)-crs(1))+2*pi);
%     L1 = norm(crs-cre) + R*mo(2*pi + mo(theta-pi/2) - mo(chis-pi/2))...
%         +R*mo(2*pi+mo(chie-pi/2) - mo(theta-pi/2));
% %     compute L2
%     ell = norm(cle-crs);
%     theta = mo(atan2(cle(2)-crs(2),cle(1)-crs(1))+2*pi);
%     theta2 = theta-pi/2 +asin(2*R/ell);
%     if isreal(theta2)==0, 
%       L2 = 9999; 
%     else
%       L2 = sqrt(ell^2 - 4*R^2) + R*mo(2*pi+mo(theta2) - mo(chis-pi/2))...
%           +R*mo(2*pi + mo(theta2+pi) - mo(chie+pi/2));
%     end
%     % compute L3
%     ell = norm(cre-cls);
%     theta = mo(atan2(cls(2)-cre(2), cls(1)-cre(1))+2*pi);
%     theta2 = acos(2*R/ell);
%     if isreal(theta2)==0,
%       L3 = 9999;
%     else
%       L3 = sqrt(ell^2 - 4*R^2) + R*mo(2*pi + mo(chis+pi/2) - mo(theta+theta2))...
%           +R*(2*pi + mo(chie-pi/2) - mo(theta+theta2-pi));
%     end
%     % compute L4
%     theta = mo(atan2(cls(2)-cle(2),cls(1)-cle(1))+2*pi);
%     L4 = norm(cls-cle) + R*mo(2*pi+mo(chis+pi/2) - mo(theta+pi/2))...
%         +R*mo(2*pi + mo(theta+pi/2) - mo(chie+pi/2));
%     % L is the minimum distance
%     [L,idx] = min([L1,L2,L3,L4]);
%     e1 = [1; 0; 0];
%     switch(idx),
%         case 1,
%             cs = crs;
%             lams = 1;
%             ce = cre;
%             lame = 1;
%             q1 = (ce-cs)/norm(ce-cs);
%             w1 = cs + R*rotz(-pi/2)*q1;
%             w2 = ce + R*rotz(-pi/2)*q1;
%         case 2,   
%             cs = crs;
%             lams = 1;
%             ce = cle;
%             lame = -1;
%             ell = norm(ce-cs);
%             theta = atan2(ce(2)-cs(2), ce(1)-cs(1)+2*pi);
%             theta2 = theta - pi/2 + asin(2*R/ell);
%             q1 = rotz(theta2+pi/2)*e1;
%             w1 = cs + R*rotz(theta2)*e1;
%             w2 = ce + R*rotz(theta2*pi)*e1;
%         case 3,
%             cs = cls;
%             lams = -1;
%             ce = cre;
%             lame = 1;
%             ell = norm(ce-cs);
%             theta = atan2(ce(2)-cs(2),ce(1)-cs(1)+2*pi);
%             theta2 = acos(2*R/ell);
%             q1 = rotz(theta+theta2-pi/2)*e1;
%             w1 = cs + R*rotz(theta+theta2)*e1;
%             w2 = ce + R*rotz(theta+theta2-pi)*e1;
%          case 4,
%             cs = cls;
%             lams = -1;
%             ce = cle;
%             lame = -1;
%             q1 = (ce-cs)/norm(ce-cs);
%             w1 = cs + R*rotz(pi/2)*q1;
%             w2 = ce + R*rotz(pi/2)*q1;
%     end
%     w3 = pe';
%     q3 = rotz(chie)*e1;
%     
%     % assign path variables
%     dubinspath.ps   = ps;
%     dubinspath.chis = chis;
%     dubinspath.pe   = pe;
%     dubinspath.chie = chie;
%     dubinspath.R    = R;
%     dubinspath.L    = L;
%     dubinspath.cs   = cs;
%     dubinspath.lams = lams;
%     dubinspath.ce   = ce;
%     dubinspath.lame = lame;
%     dubinspath.w1   = w1;
%     dubinspath.q1   = q1;
%     dubinspath.w2   = w2;
%     dubinspath.w3   = w3';
%     dubinspath.q3   = q3;
%   end
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% rotz(theta)
% %%   rotation matrix about the z axis.
% function R = rotz(theta)
%     R = [...
%         cos(theta), -sin(theta), 0;...
%         sin(theta), cos(theta), 0;...
%         0, 0, 1;...
%         ];
% end
% 
% function out = mo(angle)
%     out = mod(angle,2*pi);
% end