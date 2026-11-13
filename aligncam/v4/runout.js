/* ALIGNCAM: independent wheel-axis fit / rim-clamp runout compensation.
   Copyright (c) 2026 EDAP. Original mathematical implementation.
   No AGPL or third-party project source incorporated.
   Requires fixed camera during rotation and verified common reference.
*/
(function(root){
'use strict';
const deg=180/Math.PI;
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
const norm=v=>{const a=Math.hypot(...v);return a>1e-10?v.map(x=>x/a):null;};
const tr=m=>m[0][0]+m[1][1]+m[2][2];
const col=(m,c)=>[m[0][c],m[1][c],m[2][c]];
const mul=(a,b)=>Array.from({length:3},(_,i)=>Array.from({length:3},(_,j)=>a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j]));
const T=m=>[[m[0][0],m[1][0],m[2][0]],[m[0][1],m[1][1],m[2][1]],[m[0][2],m[1][2],m[2][2]]];
const apply=(m,v)=>m.map(r=>dot(r,v));
const angle=a=>Math.acos(clamp((tr(a)-1)/2,-1,1))*deg;
const relativeAngle=(Ra,Rb)=>angle(mul(Ra,T(Rb)));
const axisCandidate=(Ri,Rj)=>{
 const r=mul(Rj,T(Ri));
 const theta=angle(r);
 // Rodrigues skew terms become noisy near 0 or 180 degrees.
 if(theta<18||theta>162)return null;
 const v=norm([r[2][1]-r[1][2],r[0][2]-r[2][0],r[1][0]-r[0][1]]);
 if(!v)return null;
 return {axis:v,weight:Math.abs(Math.sin(theta/deg)),theta};
};
function fit(poses){
 const valid=(poses||[]).filter(R=>Array.isArray(R)&&R.length===3&&R.every(row=>Array.isArray(row)&&row.length===3&&row.every(Number.isFinite)));
 if(valid.length<4)return {ok:false,reason:'Collect at least 4 different wheel positions',samples:valid.length};
 let candidates=[],maxSpan=0;
 for(let i=0;i<valid.length;i++)for(let j=i+1;j<valid.length;j++){
  const span=relativeAngle(valid[j],valid[i]);maxSpan=Math.max(maxSpan,span);
  const a=axisCandidate(valid[i],valid[j]);if(a)candidates.push(a);
 }
 if(maxSpan<70||candidates.length<4)return {ok:false,reason:'Rotate the wheel through at least 70 degrees total',samples:valid.length,spanDeg:maxSpan};
 // Axle direction is undirected. Orient all candidates towards the optical-facing marker normal.
 const front=col(valid[0],2),orient=a=>dot(a,front)<0?a.map(x=>-x):a;
 const axes=candidates.map(p=>({...p,axis:orient(p.axis)}));
 let ref=norm(axes.reduce((acc,p)=>acc.map((x,k)=>x+p.axis[k]*p.weight),[0,0,0]));
 if(!ref)return {ok:false,reason:'Axle fit is degenerate',samples:valid.length,spanDeg:maxSpan};
 // Angular consensus: reject pairwise pose outliers rather than smoothing incorrect axes.
 let filtered=axes.filter(p=>Math.acos(clamp(dot(ref,p.axis),-1,1))*deg<8);
 if(filtered.length<4)return {ok:false,reason:'Inconsistent target poses; focus and camera mount must be stable',samples:valid.length,spanDeg:maxSpan};
 ref=norm(filtered.reduce((acc,p)=>acc.map((x,k)=>x+p.axis[k]*p.weight),[0,0,0]));
 const scatter=Math.sqrt(filtered.reduce((s,p)=>s+(Math.acos(clamp(dot(ref,p.axis),-1,1))*deg)**2,0)/filtered.length);
 const axisBoard=norm(apply(T(valid[0]),ref));
 const ok=!!axisBoard&&scatter<=2.5&&filtered.length>=4;
 return {ok,reason:ok?'Wheel rotation axis calibrated':'Runout samples disagree; repeat with better marker focus',samples:valid.length,spanDeg:maxSpan,scatterDeg:scatter,pairs:filtered.length,axisBoard,axisCamera:ref};
}
function axisInCamera(R,axisBoard){const v=norm(apply(R,axisBoard));return v||null;}
function relativeYaw(R,axisBoard,refR){
 const a=axisInCamera(R,axisBoard);
 if(!a||!refR)return null;
 const x=dot(col(refR,0),a),z=dot(col(refR,2),a);
 return Math.atan2(x,z)*deg;
}
root.AlignRunout={fit,axisInCamera,relativeYaw,relativeAngle,matMul:mul,transpose:T,axisCandidate};
if(typeof module==='object'&&module.exports)module.exports=root.AlignRunout;
})(typeof window!=='undefined'?window:globalThis);
