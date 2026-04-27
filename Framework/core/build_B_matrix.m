function B = build_B_matrix(Lx, Lyi, Lyo, cT)
% BUILD_B_MATRIX  Construct the 4x6 control effectiveness matrix.
%
%   Maps individual motor thrusts T = [T1...T6]' (all ≥ 0) to virtual
%   controls v = [Fz; L; M; N]:
%       Fz  - net propulsive force along body z-axis [N]  (negative = upward)
%       L   - roll moment about body x-axis [Nm]
%       M   - pitch moment about body y-axis [Nm]
%       N   - yaw moment about body z-axis [Nm]
%
%   Motor layout and spin assignments (PPNNPN / B2 configuration):
%     Motor  x-pos   y-pos   spin    yaw sign
%       1    +Lx     -Lyi    CCW     -cT
%       2    +Lx     +Lyi    CCW     -cT
%       3     0      -Lyo    CW      +cT
%       4     0      +Lyo    CW      +cT
%       5    -Lx     -Lyi    CCW     -cT
%       6    -Lx     +Lyi    CW      +cT
%
%   Inputs:
%     Lx   - fore/aft arm length [m]
%     Lyi  - inner lateral arm length [m]
%     Lyo  - outer lateral arm length [m]
%     cT   - moment-to-thrust ratio [-]
%
%   Output:
%     B    - [4x6] control effectiveness matrix
%
%   Convention: thrust positive, Fz row negative (thrust opposes gravity
%   in NED, where z is down).

%         M1      M2      M3      M4      M5      M6
B = [    -1      -1      -1      -1      -1      -1;   % Fz  [N]
      -Lyi     Lyi    -Lyo     Lyo    -Lyi     Lyi;   % L   [Nm]
        Lx      Lx       0       0     -Lx     -Lx;   % M   [Nm]
       -cT     -cT      cT      cT     -cT      cT];  % N   [Nm]
end
