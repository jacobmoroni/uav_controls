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
  if ell<2*R,
      disp('The distance between nodes must be larger than 2R.');
      dubinspath = [];
  else
    
    ps   = ;
    chis = ;
    pe   = ;
    chie = ;
    

    crs = ;
    cls = ;
    cre = ;
    cle = ;
    
   
    % compute L1
    theta = ;
    L1 = ;
    % compute L2
    ell = ;
    theta = ;
    theta2 = ;
    if isreal(theta2)==0, 
      L2 = 9999; 
    else
      L2 = ;
    end
    % compute L3
    ell = ;
    theta = ;
    theta2 = ;
    if isreal(theta2)==0,
      L3 = 9999;
    else
      L3 = ;
    end
    % compute L4
    theta = ;
    L4 = ;
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    e1 = [1; 0; 0];
    switch(idx),
        case 1,
            cs = ;
            lams = ;
            ce = ;
            lame = ;
            q1 = ;
            w1 = ;
            w2 = ;
        case 2,   
            cs = ;
            lams = ;
            ce = ;
            lame = ;
            ell = ;
            theta = ;
            theta2 = ;
            q1 = ;
            w1 = ;
            w2 = ;
        case 3,
            cs =;
            lams =;
            ce = ;
            lame = ;
            ell = ;
            theta = ;
            theta2 = ;
            q1 = ;
            w1 = ;
            w2 = ;
         case 4,
            cs = ;
            lams = ;
            ce = ;
            lame = ;
            q1 = ;
            w1 = ;
            w2 = ;
    end
    w3 = ;
    q3 = ;
    
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% rotz(theta)
%%   rotation matrix about the z axis.
function R = rotz(theta)
    R = [...
        cos(theta), -sin(theta), 0;...
        sin(theta), cos(theta), 0;...
        0, 0, 1;...
        ];
end
