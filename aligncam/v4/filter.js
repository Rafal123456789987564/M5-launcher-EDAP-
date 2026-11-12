/*
 * AlignCam V4.2 multi-stage measurement quality control.
 * Copyright (c) 2026 EDAP. All rights reserved.
 * Original implementation of public mathematical methods, not proprietary automotive binaries.
 * Hampel / MAD outlier rejection -> innovation-gated Kalman -> adaptive One-Euro low-pass -> stability gate.
 */
(function(root){
'use strict';
const median=a=>{if(!a.length)return NaN;const s=a.slice().sort((x,y)=>x-y),k=s.length>>1;return s.length%2?s[k]:(s[k-1]+s[k])/2;};
const clamp=(x,l,h)=>Math.min(h,Math.max(l,x));
const std=a=>{if(!a.length)return Infinity;const m=a.reduce((s,x)=>s+x,0)/a.length;return Math.sqrt(a.reduce((s,x)=>s+(x-m)**2,0)/a.length);};
class AngleChannel{
 constructor(){this.clear();}
 clear(){this.samples=[];this.outputs=[];this.x=null;this.v=0;this.P=[.09,0,0,1];this.smooth=null;this.deriv=0;this.lastT=null;this.pending=null;this.pendingCount=0;this.goodStreak=0;this.lastReason='STARTING';}
 update(value,t,meta={}){
  if(!Number.isFinite(value))return this.invalidate('NO MEASUREMENT');
  if(this.lastT!==null&&(t-this.lastT>1.2||t<this.lastT))this.clear();
  const dt=clamp(this.lastT===null?.145:t-this.lastT,.04,.5);
  this.lastT=t;
  const pixelErr=Number.isFinite(meta.pixelErr)?meta.pixelErr:2;
  const markers=Number.isFinite(meta.markers)?meta.markers:7;
  const R=clamp((.055+.07*pixelErr+2/Math.max(4,markers))**2,.018,.55);
  const recent=this.samples.slice(-9);
  if(recent.length>=5){
   const center=median(recent),mad=median(recent.map(x=>Math.abs(x-center)));
   const gate=Math.max(.52,4*1.4826*mad,.32+Math.abs(this.v)*dt*3);
   if(Math.abs(value-center)>gate){
    if(this.pending!==null&&Math.abs(value-this.pending)<.32){this.pendingCount++;this.pending=this.pending*.5+value*.5;}
    else{this.pending=value;this.pendingCount=1;}
    if(this.pendingCount<3){this.goodStreak=0;this.lastReason='JUMP DETECTED — VERIFY MOVEMENT';return {ok:false,reason:this.lastReason};}
    this.clear();this.lastT=t;
   }else{this.pending=null;this.pendingCount=0;}
  }
  if(this.x===null){this.x=value;this.v=0;this.smooth=value;this.P=[R,0,0,1];this.goodStreak=1;this.samples.push(value);this.outputs.push(value);return{ok:false,reason:'ACQUIRING'};}
  const q=.65,pPred=this.x+this.v*dt;
  const p00=this.P[0]+dt*(this.P[1]+this.P[2])+dt*dt*this.P[3]+q*dt**4/4;
  const p01=this.P[1]+dt*this.P[3]+q*dt**3/2;
  const p10=this.P[2]+dt*this.P[3]+q*dt**3/2;
  const p11=this.P[3]+q*dt*dt;
  const innovation=value-pPred,S=p00+R;
  if(this.samples.length>=4&&Math.abs(innovation)>Math.max(.65,3.7*Math.sqrt(S))){this.goodStreak=0;this.lastReason='KALMAN INNOVATION REJECTED';return{ok:false,reason:this.lastReason};}
  const k0=p00/S,k1=p10/S;
  this.x=pPred+k0*innovation;
  this.v=clamp(this.v+k1*innovation,-15,15);
  this.P=[Math.max(1e-8,(1-k0)*p00),(1-k0)*p01,p10-k1*p00,Math.max(1e-8,p11-k1*p01)];
  const alpha=(hz,dt)=>{const tau=1/(2*Math.PI*hz);return dt/(tau+dt);};
  const dv=(this.x-this.smooth)/dt;
  this.deriv+=alpha(1,dt)*(dv-this.deriv);
  const cutoff=.55+.24*Math.abs(this.deriv);
  this.smooth+=alpha(cutoff,dt)*(this.x-this.smooth);
  this.samples.push(value);if(this.samples.length>11)this.samples.shift();
  this.outputs.push(this.smooth);if(this.outputs.length>11)this.outputs.shift();
  this.goodStreak++;
  const scatter=std(this.samples.slice(-7)),outScatter=std(this.outputs.slice(-7));
  const stable=this.goodStreak>=6&&outScatter<=.19&&scatter<=.42;
  const moving=this.goodStreak>=6&&Math.abs(this.v)>.16&&scatter<=1;
  this.lastReason=stable?'STABLE':moving?'MOVING':'SETTLING';
  if(!stable&&!moving)return{ok:false,reason:this.lastReason,scatter};
  return{ok:true,value:this.smooth,stable,moving,scatter,reason:this.lastReason};
 }
 invalidate(reason){this.goodStreak=0;this.lastReason=reason;return{ok:false,reason};}
}
class QualityPipeline{
 constructor(){this.yaw=new AngleChannel();this.camber=new AngleChannel();this.level=new AngleChannel();this.orientation=new AngleChannel();}
 reset(){this.yaw.clear();this.camber.clear();this.level.clear();this.orientation.clear();}
 process(raw,t=Date.now()/1000){
  if(!raw)return null;
  const accepted=(raw.status==='GOOD'||raw.status==='FAIR')&&raw.fit?.valid&&raw.fit.inliers>=6&&raw.fit.error<=2.45;
  if(!accepted){
   for(const channel of [this.yaw,this.camber,this.level,this.orientation])channel.invalidate('TRACKING LOST');
   return{...raw,relativeYaw:null,camber:null,level:null,levelText:'UNAVAILABLE',filterStable:false,filterState:raw.status,filterMoving:false,filterScatter:null,status:(raw.status==='GOOD'||raw.status==='FAIR')?'LOW QUALITY':raw.status};
  }
  const quality={pixelErr:raw.fit.error,markers:raw.fit.inliers};
  const orientation=Number.isFinite(raw.fit.yaw)?this.orientation.update(raw.fit.yaw,t,quality):{ok:false,reason:'NO POSE'};
  const yaw=Number.isFinite(raw.relativeYaw)?this.yaw.update(raw.relativeYaw,t,quality):{ok:false,reason:'NO REFERENCE'};
  const camber=Number.isFinite(raw.camber)?this.camber.update(raw.camber,t,quality):{ok:false,reason:'NO GRAVITY'};
  const level=Number.isFinite(raw.level)?this.level.update(raw.level,t,quality):{ok:false,reason:'NO GRAVITY'};
  const check=Number.isFinite(raw.relativeYaw)?yaw:orientation;
  const blocking=['JUMP DETECTED','INNOVATION REJECTED'].some(z=>String(check.reason).includes(z));
  const out={...raw,relativeYaw:yaw.ok?yaw.value:null,camber:camber.ok?camber.value:null,level:level.ok?level.value:null};
  out.levelText=level.ok?level.value.toFixed(1)+'°'+(level.value<=3?' ✓':' !'):'UNAVAILABLE';
  out.filterState=check.reason||'SETTLING';out.filterScatter=check.scatter??null;out.filterStable=!!check.stable;out.filterMoving=!!check.moving;
  if(!check.ok){out.status=blocking?'UNSTABLE':'STABILIZING';out.note=blocking?'SUDDEN ANGLE JUMP — CHECK LIGHTING OR MOUNT':'Filtering: '+check.reason+'. Keep both phones steady.';}
  else if(!out.filterStable){out.status='MOVING';out.note='Tracking movement • '+raw.fit.inliers+'/12 IDs • '+raw.fit.error.toFixed(1)+' px';}
  else{out.status=raw.status;out.note='Stable • '+raw.fit.inliers+'/12 IDs • '+raw.fit.error.toFixed(1)+' px • '+(Number.isFinite(out.filterScatter)?out.filterScatter.toFixed(2):'—')+'° noise';}
  return out;
 }
}
root.AlignFilters={AngleChannel,QualityPipeline,median,std};
if(typeof module!=='undefined'&&module.exports)module.exports=root.AlignFilters;
})(typeof window!=='undefined'?window:globalThis);
