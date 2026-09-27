# 光滑化 max() 的 NLP.mod 文件
param alpha := 60;  # 光滑参数

param PV{i in 1..18};
param Nobs := PV[16];
param PPP{i in 1..Nobs, j in 1..4, k in 1..2};
param Area{i in 1..Nobs};
param Nfe := PV[7];

#====================================
var tf >= 0;
var x{i in 1..Nfe};
var y{i in 1..Nfe};
var theta{i in 1..Nfe};
var v{i in 1..Nfe};
var a{i in 1..Nfe};
var phy{i in 1..Nfe};
var w{i in 1..Nfe};

var AX{i in 1..Nfe-1};
var AY{i in 1..Nfe-1};
var BX{i in 1..Nfe-1};
var BY{i in 1..Nfe-1};
var CX{i in 1..Nfe-1};
var CY{i in 1..Nfe-1};
var DX{i in 1..Nfe-1};
var DY{i in 1..Nfe-1};

var dt{i in 1..Nfe-1} >= 0;
var s{i in 1..Nfe-1};
var splus{i in 1..Nfe-1};
var sminus{i in 1..Nfe-1};
var up{i in 1..Nfe-1};
var down{i in 1..Nfe-1};
var left{i in 1..Nfe-1};
var right{i in 1..Nfe-1};
var k{i in 1..Nfe-1};

#============================================
param a_max := PV[10];
param v_max := PV[8];
param w_max := PV[11];
param phy_max := PV[9];

param lw := PV[12];
param lr := PV[14];
param hlb := PV[15];
param LF := PV[13];
param lb := hlb * 2;
param max_dt := PV[17];
param min_dt := PV[18];

#==============（时间约束）============
s.t. define_tf: tf = sum{i in 1..Nfe-1} dt[i];
s.t. time_bound1{i in 1..Nfe-1}: min_dt <= dt[i] <= max_dt;
s.t. time_bound2: tf <= 35;

# ============（代价函数）=================
minimize objective_: tf;

#===========（车辆动力学约束）===============
s.t. DIFF_dxdt{i in 1..Nfe-1}: x[i+1] = x[i] + v[i]*dt[i]*cos(theta[i]);
s.t. DIFF_dydt{i in 1..Nfe-1}: y[i+1] = y[i] + v[i]*dt[i]*sin(theta[i]);
s.t. DIFF_dvdt{i in 1..Nfe-1}: v[i+1] = v[i] + dt[i]*a[i];
s.t. DIFF_dthetadt{i in 1..Nfe-1}: theta[i+1] = theta[i] + dt[i]*v[i]*tan(phy[i])/lw;
s.t. DIFF_dphydt{i in 1..Nfe-1}: phy[i+1] = phy[i] + dt[i]*w[i];

# =============（车体边界约束）==============
s.t. Bounds_AX{i in 1..Nfe-1}: 0 <= AX[i] <= 30;
s.t. Bounds_BX{i in 1..Nfe-1}: 0 <= BX[i] <= 30;
s.t. Bounds_CX{i in 1..Nfe-1}: 0 <= CX[i] <= 30;
s.t. Bounds_DX{i in 1..Nfe-1}: 0 <= DX[i] <= 30;
s.t. Bounds_AY{i in 1..Nfe-1}: 0 <= AY[i] <= 30;
s.t. Bounds_BY{i in 1..Nfe-1}: 0 <= BY[i] <= 30;
s.t. Bounds_CY{i in 1..Nfe-1}: 0 <= CY[i] <= 30;
s.t. Bounds_DY{i in 1..Nfe-1}: 0 <= DY[i] <= 30;

# =============（具身足迹相关变量）=====================
s.t. define_kappa{i in 1..Nfe-1}: k[i] = tan(phy[i])/lw;
s.t. define_s{i in 1..Nfe-1}: s[i] = v[i]*dt[i];

# 光滑化 splus / sminus
s.t. define_splus{i in 1..Nfe-1}: splus[i] = (1/alpha)*log(1+exp(alpha*s[i]));          # 原先为splus = max(s,0)
s.t. define_sminus{i in 1..Nfe-1}: sminus[i] = (1/alpha)*log(1+exp(alpha*(-s[i])));     # 原先为sminus = min(-s,0)

# 具身足迹外扩尺度
s.t. define_up{i in 1..Nfe-1}: up[i] = splus[i] + hlb*abs(k[i])*splus[i];       # up与down均不做处理

s.t. define_down{i in 1..Nfe-1}: down[i] = sminus[i] + hlb*abs(k[i])*sminus[i];

s.t. define_left{i in 1..Nfe-1}:            # 原先为max(a,b)+max(c,d)，均用log-sum-exp做平滑处理
    left[i] = (1/alpha)*log(exp(alpha*(-lr*k[i]*splus[i])) + exp(alpha*((LF+0.5*splus[i])*k[i]*splus[i])))
             + (1/alpha)*log(exp(alpha*(-LF*k[i]*sminus[i])) + exp(alpha*((lr+0.5*sminus[i])*k[i]*sminus[i])));

s.t. define_right{i in 1..Nfe-1}:
    right[i] = (1/alpha)*log(exp(alpha*(lr*k[i]*splus[i])) + exp(alpha*(-(LF+0.5*splus[i])*k[i]*splus[i])))
              + (1/alpha)*log(exp(alpha*(LF*k[i]*sminus[i])) + exp(alpha*(-(lr+0.5*sminus[i])*k[i]*sminus[i])));


#================（两点边值约束）================
s.t. Init_X:
x[1] = PV[1];
s.t. Init_Y:
y[1] = PV[2];
s.t. Init_Theta:
theta[1] = PV[3];
s.t. End_X:
x[Nfe] = PV[4];
s.t. End_Y:
y[Nfe] = PV[5];
s.t. End_Theta:
theta[Nfe] = PV[6];

s.t. Init_W:
w[1] = 0;
s.t. Init_A:
a[1] = 0;
s.t. Init_Phy:
phy[1] = 0;
s.t. Init_v:
v[1] = 0;
s.t. End_W:
w[Nfe] = 0;
s.t. End_A:
a[Nfe] = 0;
s.t. End_Phy:
phy[Nfe] = 0;
s.t. End_v:
v[Nfe] = 0;

#==============（车辆动力学约束）======
s.t. Bonds_v {i in {1..Nfe}}:
-v_max <= v[i] <= v_max;
s.t. Bonds_a {i in {1..Nfe}}:
-a_max <= a[i] <= a_max;
s.t. Bonds_phy {i in {1..Nfe}}:
-phy_max <= phy[i] <= phy_max;
s.t. Bonds_w {i in {1..Nfe}}:
-w_max <= w[i] <= w_max;

#=================（具身三约束）===========
# 具身足迹有效转角限制
s.t. EF_arc_bound{i in {1..(Nfe-1)}}:
    abs(k[i]) * (splus[i] + sminus[i]) <= 1.5708;

# 前进条件1
s.t. EF_forward_cond1{i in {1..(Nfe-1)}}:
    (1 + hlb * abs(k[i])) * tan(abs(k[i]) * splus[i]) <= lr * abs(k[i]);
# 前进条件2
s.t. EF_forward_cond2{i in {1..(Nfe-1)}}:
    abs(k[i]) * LF * tan(abs(k[i]) * splus[i]) <= 1 + hlb * abs(k[i]);

# 倒车条件1
s.t. EF_reverse_cond1{i in {1..(Nfe-1)}}:
    (1 + hlb * abs(k[i])) * tan(abs(k[i]) * sminus[i]) <= LF * abs(k[i]);
# 倒车条件2
s.t. EF_reverse_cond2{i in {1..(Nfe-1)}}:
    abs(k[i]) * lr * tan(abs(k[i]) * sminus[i]) <= 1 + hlb * abs(k[i]);


#=============（具身盒子顶点的计算）=====================
s.t. RELATIONSHIP_AX{i in {1..(Nfe-1)}}:
    AX[i] = x[i] + (LF + up[i]) * cos(theta[i]) - (hlb + left[i]) * sin(theta[i]);
s.t. RELATIONSHIP_BX{i in {1..(Nfe-1)}}:
    BX[i] = x[i] + (LF + up[i]) * cos(theta[i]) + (hlb + right[i]) * sin(theta[i]);
s.t. RELATIONSHIP_CX{i in {1..(Nfe-1)}}:
    CX[i] = x[i] - (lr + down[i]) * cos(theta[i]) + (hlb + right[i]) * sin(theta[i]);
s.t. RELATIONSHIP_DX{i in {1..(Nfe-1)}}:
    DX[i] = x[i] - (lr + down[i]) * cos(theta[i]) - (hlb + left[i]) * sin(theta[i]);
s.t. RELATIONSHIP_AY{i in {1..(Nfe-1)}}:
    AY[i] = y[i] + (LF + up[i]) * sin(theta[i]) + (hlb + left[i]) * cos(theta[i]);
s.t. RELATIONSHIP_BY{i in {1..(Nfe-1)}}:
    BY[i] = y[i] + (LF + up[i]) * sin(theta[i]) - (hlb + right[i]) * cos(theta[i]);
s.t. RELATIONSHIP_CY{i in {1..(Nfe-1)}}:
    CY[i] = y[i] - (lr + down[i]) * sin(theta[i]) - (hlb + right[i]) * cos(theta[i]);
s.t. RELATIONSHIP_DY{i in {1..(Nfe-1)}}:
    DY[i] = y[i] - (lr + down[i]) * sin(theta[i]) + (hlb + left[i]) * cos(theta[i]);


#===================（避障约束，使用三角面积法）=====================================
s.t. eq_PPPoutsideABCD {i in {2..(Nfe-1)}, nn in {1..Nobs}, jj in {1..4}}:
(abs((AX[i] - PPP[nn,jj,1])*(BY[i] - PPP[nn,jj,2]) - (AY[i] - PPP[nn,jj,2])*(BX[i] - PPP[nn,jj,1])) * 0.5 + abs((BX[i] - PPP[nn,jj,1])*(CY[i] - PPP[nn,jj,2]) - (BY[i] - PPP[nn,jj,2])*(CX[i] - PPP[nn,jj,1])) * 0.5 + abs((CX[i] - PPP[nn,jj,1])*(DY[i] - PPP[nn,jj,2]) - (CY[i] - PPP[nn,jj,2])*(DX[i] - PPP[nn,jj,1])) * 0.5 + abs((DX[i] - PPP[nn,jj,1])*(AY[i] - PPP[nn,jj,2]) - (DY[i] - PPP[nn,jj,2])*(AX[i] - PPP[nn,jj,1])) * 0.5) >= (LF + lr + up[i] + down[i]) * (lb + right[i] + left[i]) + 0.1;

s.t. eq_AoutsidePRECTANGLEPPP {i in {2..(Nfe-1)}, nn in {1..Nobs}}:
(abs((PPP[nn,1,1] - AX[i])*(PPP[nn,2,2] - AY[i]) - (PPP[nn,1,2] - AY[i])*(PPP[nn,2,1] - AX[i])) * 0.5 + abs((PPP[nn,2,1] - AX[i])*(PPP[nn,3,2] - AY[i]) - (PPP[nn,2,2] - AY[i])*( PPP[nn,3,1] - AX[i])) * 0.5 + abs((PPP[nn,3,1] - AX[i])*( PPP[nn,4,2] - AY[i]) - (PPP[nn,3,2] - AY[i])*( PPP[nn,4,1] - AX[i])) * 0.5 + abs((PPP[nn,4,1] - AX[i])*( PPP[nn,1,2] - AY[i]) - (PPP[nn,4,2] - AY[i])*( PPP[nn,1,1] - AX[i])) * 0.5) >= Area[nn];

s.t. eq_BoutsidePRECTANGLEPPP {i in {2..(Nfe-1)}, nn in {1..Nobs}}:
(abs((PPP[nn,1,1] - BX[i])*(PPP[nn,2,2] - BY[i]) - (PPP[nn,1,2] - BY[i])*(PPP[nn,2,1] - BX[i])) * 0.5 + abs((PPP[nn,2,1] - BX[i])*(PPP[nn,3,2] - BY[i]) - (PPP[nn,2,2] - BY[i])*( PPP[nn,3,1] - BX[i])) * 0.5 + abs((PPP[nn,3,1] - BX[i])*( PPP[nn,4,2] - BY[i]) - (PPP[nn,3,2] - BY[i])*( PPP[nn,4,1] - BX[i])) * 0.5 + abs((PPP[nn,4,1] - BX[i])*( PPP[nn,1,2] - BY[i]) - (PPP[nn,4,2] - BY[i])*( PPP[nn,1,1] - BX[i])) * 0.5) >= Area[nn];

s.t. eq_CoutsidePRECTANGLEPPP {i in {2..(Nfe-1)}, nn in {1..Nobs}}:
(abs((PPP[nn,1,1] - CX[i])*(PPP[nn,2,2] - CY[i]) - (PPP[nn,1,2] - CY[i])*(PPP[nn,2,1] - CX[i])) * 0.5 + abs((PPP[nn,2,1] - CX[i])*(PPP[nn,3,2] - CY[i]) - (PPP[nn,2,2] - CY[i])*( PPP[nn,3,1] - CX[i])) * 0.5 + abs((PPP[nn,3,1] - CX[i])*( PPP[nn,4,2] - CY[i]) - (PPP[nn,3,2] - CY[i])*( PPP[nn,4,1] - CX[i])) * 0.5 + abs((PPP[nn,4,1] - CX[i])*( PPP[nn,1,2] - CY[i]) - (PPP[nn,4,2] - CY[i])*( PPP[nn,1,1] - CX[i])) * 0.5) >= Area[nn];

s.t. eq_DoutsidePRECTANGLEPPP {i in {2..(Nfe-1)}, nn in {1..Nobs}}:
(abs((PPP[nn,1,1] - DX[i])*(PPP[nn,2,2] - DY[i]) - (PPP[nn,1,2] - DY[i])*(PPP[nn,2,1] - DX[i])) * 0.5 + abs((PPP[nn,2,1] - DX[i])*(PPP[nn,3,2] - DY[i]) - (PPP[nn,2,2] - DY[i])*( PPP[nn,3,1] - DX[i])) * 0.5 + abs((PPP[nn,3,1] - DX[i])*( PPP[nn,4,2] - DY[i]) - (PPP[nn,3,2] - DY[i])*( PPP[nn,4,1] - DX[i])) * 0.5 + abs((PPP[nn,4,1] - DX[i])*( PPP[nn,1,2] - DY[i]) - (PPP[nn,4,2] - DY[i])*( PPP[nn,1,1] - DX[i])) * 0.5) >= Area[nn];

#==================（加载数据）=============================
data;
param PV := include PV;
param PPP := include PPP;
param Area := include Area;
