{ Utils v1.0

  Copyright (C) 2015 Paulo Costa

  All rights reserved.
  This file is licensed under the BSD License. See http://www.opensource.org/licenses/bsd-license.php
  Redistribution and use in source and binary forms, with or without modification, are permitted provided that
  the following conditions are met:
  Redistributions of source code must retain the above copyright notice, this list of conditions and the
  following disclaimer.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
}

unit Utils;

{$mode objfpc}{$H+}

interface

uses Classes, sysutils, Math, graphics, Grids;

procedure ParseString(s,sep: string; sl: TStrings);
procedure LoadGridFromfile(SG: TStringGrid; fname: string);
procedure SaveGridTofile(SG: TStringGrid; fname: string);
procedure WriteStringToGrid(SG: TStringGrid; vname: string; icol: longword; wval: string);
procedure WriteFloatToGrid(SG: TStringGrid; vname: string; icol: longword; wval: double);

function GetStringFromGrid(SG: TStringGrid; vname: string; icol: longword; defval: string = ''): string;
function GetFloatFromGrid(SG: TStringGrid; vname: string; icol: longword; defval: Double = 0): Double;

function FMod(x,d: double): double;
function DiffAngle(a1,a2: double): double;
function Dist(x,y: double): double;
function ATan2(y,x: double): double;
function ACos2(y,x: double): double;
function ASin2(y,x: double): double;
function Sign(a: double): double;
function Sat(a,limit: double): double;

function IncWrap(var v: integer; size: integer; step:integer=1): integer;
function DecWrap(var v: integer; size: integer; step:integer=1): integer;
procedure SwapInts(var v1, v2: integer);
function Middle(a,b,c: integer): integer;

function Rad(xw: double):double;
function deg(xw: double):double;

procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
procedure RotateAndTranslate(var rx,ry: double; px,py,tx,ty,teta: double);
procedure RotateAroundPoint(var rx,ry: double; px,py,cx,cy,teta: double);

function InternalProductCosine(v1x,v1y,v2x,v2y: double): double;

function NormalizeAngle(ang: double): double;

function LinInterp(x0, x1, y0, y1, x: double): double;
//function BiLinInterp(Surf: matrix; xmin, xmax, ymin, ymax, x,y: double): double;

procedure RotateCov( const incov_x,incov_y,incov_xy: double; out cov_x,cov_y, cov_xy: double; teta: double);


//type
//  QSortCmpFunc=function (var a,b): integer;
//  pbyte=^byte;

//procedure QSort(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc);

implementation

uses StrUtils;


procedure ParseString(s,sep: string; sl: TStrings);
var p,i,last: integer;
begin
  sl.Clear;
  last:=1;
  for i:=1 to length(s) do begin
    p:=Pos(s[i],sep);
    if p>0 then begin
      if i<>last then
        sl.add(copy(s,last,i-last));
      last:=i+1;
    end;
  end;
  if last<=length(s) then
    sl.add(copy(s,last,length(s)-last+1));
end;

procedure LoadGridFromfile(SG: TStringGrid; fname: string);
var SL: TStringList;
    i, icol, tabpos, ntp: integer;
    s, sub: string;
begin
  sub := #9;
  SL := TStringList.Create;
  try
    SL.LoadFromFile(fname);
    for i:=0 to SL.Count-1 do begin
      if i >= SG.RowCount then break;
      icol := 0;
      ntp := 1;
      tabpos := 1;
      while ntp <> 0 do begin
        ntp := PosEx(sub, SL.Strings[i], tabpos);
        if ntp = 0 then begin
          s := copy(SL.Strings[i], tabpos, length(SL.Strings[i]) - tabpos + 1);
        end else begin
          s := copy(SL.Strings[i], tabpos, ntp - tabpos);
        end;
        tabpos := ntp + 1;
        if icol >= SG.colCount then break;
        SG.Cells[icol, i] := trim(s);
        inc(icol);
      end;
    end;
  finally
    SL.Free;
  end;
end;

procedure SaveGridTofile(SG: TStringGrid; fname: string);
var SL: TStringList;
    i, icol: integer;
    s, sub: string;
begin
  sub := #9;
  SL := TStringList.Create;
  try
    for i:=0 to SG.RowCount-1 do begin
      s := SG.Cells[0, i];
      for iCol:=1 to SG.ColCount-1 do begin
        s := s + sub + SG.Cells[icol, i];
      end;
      SL.Add(s);
    end;
    SL.SaveToFile(fname);
  finally
    SL.Free;
  end;
end;

procedure WriteStringToGrid(SG: TStringGrid; vname: string; icol: longword; wval: string);
var i: integer;
begin
  if icol >= longword(SG.ColCount) then exit;
  for i := 0  to SG.RowCount-1 do begin
    if SG.Cells[0, i] = vname then begin
      SG.Cells[icol, i] := wval;
      exit;
    end;
  end;
end;


procedure WriteFloatToGrid(SG: TStringGrid; vname: string; icol: longword; wval: double);
begin
  WriteStringToGrid(SG, vname, icol, format('%.5g',[wval]));
end;



function GetStringFromGrid(SG: TStringGrid; vname: string; icol: longword; defval: string = ''): string;
var i: integer;
begin
  result := defval;
  if integer(icol) >= SG.ColCount then exit;
  for i := 0  to SG.RowCount-1 do begin
    if SG.Cells[0, i] = vname then begin
      result := SG.Cells[icol, i];
      exit;
    end;
  end;
end;

function GetIntFromGrid(SG: TStringGrid; vname: string; icol: longword; defval: integer = -1): integer;
begin
  result := StrToIntDef(GetStringFromGrid(SG, vname, icol, ''), defval);
end;

function GetFloatFromGrid(SG: TStringGrid; vname: string; icol: longword; defval: Double = 0): Double;
begin
  result := StrToFloatDef(GetStringFromGrid(SG, vname, icol, ''), defval);
end;


// ---------------------------------------------------------
//     Math functions

function Dist(x,y: double): double;
begin
  result:=sqrt(x*x+y*y);
end;

function FMod(x,d: double): double;
begin
  result:=Frac(x/d)*d;
end;

function DiffAngle(a1,a2: double): double;
begin
  result:=a1-a2;
  if result<0 then begin
    result:=-FMod(-result,2*Pi);
    if result<-Pi then result:=result+2*Pi;
  end else begin
    result:=FMod(result,2*Pi);
    if result>Pi then result:=result-2*Pi;
  end;
end;

function ATan2(y,x: double): double;
var ax,ay: double;
begin
  ax:=Abs(x);
  ay:=Abs(y);

  if (ax<1e-10) and (ay<1e-10) then begin;
    result:=0.0;
    exit;
  end;
  if ax>ay then begin
    if x<0 then begin
      result:=ArcTan(y/x)+pi;
      if result>pi then result:=result-2*pi;
    end else begin
      result:=ArcTan(y/x);
    end;
  end else begin
    if y<0 then begin
      result:=ArcTan(-x/y)-pi/2
    end else begin
      result:=ArcTan(-x/y)+pi/2;
    end;
  end;
end;

function ACos2(y, x: double): double;
begin
  if y >= 0 then begin
    result := arccos(max(-1, min(1, x)));
  end else begin
    result := -arccos(max(-1, min(1, x)));
  end;
end;


function ASin2(y, x: double): double;
begin
  if x >= 0 then begin
    result := arcsin(max(-1, min(1, y)));
  end else if y > 0 then begin // 2nd Quadrant
    result := pi + arcsin(max(-1, min(1, y)));
  end else begin               // 3rd Quadrant
    result := -pi + arcsin(max(-1, min(1, y)));
  end;
end;


function Sign(a: double): double;
begin
  if a > 0 then begin
    result := 1;
  end else if a < 0 then begin
    result := -1;
  end else begin
    result := 0;
  end;
end;

function Sat(a,limit: double): double;
begin
 if a>limit then a:=limit;
 if a<-limit then a:=-limit;
 result:=a;
end;


function Rad(xw: double):double;
begin
  result:=xw*(pi/180);
end;

function deg(xw: double):double;
begin
  result:=xw*(180/pi);
end;

{
procedure QSortSwapElem(a,b: pbyte; size: integer);
var i: integer;
    tmp: byte;
begin
  if a=b then exit;
  for i:=0 to size-1 do begin
    tmp:=a^;
    a^:=b^;
    b^:=tmp;
    inc(a);
    inc(b);
  end;
end;

procedure QSortSub(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc; iLo, iHi: integer);
var Lo,Hi: integer;
    MidPtr: pbyte;
begin
  Lo := iLo;
  Hi := iHi;
  MidPtr := pbyte(PtrUInt(base)+PtrUInt(((iLo + iHi) div 2)*size_elem));
  repeat
    while func(pbyte(PtrUInt(base)+PtrUInt(Lo*size_elem))^,MidPtr^)<0 do Inc(Lo);
    while func(pbyte(PtrUInt(base)+PtrUInt(Hi*size_elem))^,MidPtr^)>0 do Dec(Hi);
    if Lo <= Hi then
    begin
      QSortSwapElem(
        pbyte(PtrUInt(base)+PtrUInt(Lo*size_elem)),
        pbyte(PtrUInt(base)+PtrUInt(Hi*size_elem)),
        size_elem);
      Inc(Lo);
      Dec(Hi);
    end;
  until Lo > Hi;
  if Hi > iLo then QSortSub(base,num_elem,size_elem,func, iLo, Hi);
  if Lo < iHi then QSortSub(base,num_elem,size_elem,func, Lo, iHi);
end;

procedure QSort(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc);
begin
  if num_elem<2 then exit;
  QSortSub(base,num_elem,size_elem,func,0,num_elem-1);
end;
}


function InternalProductCosine(v1x,v1y,v2x,v2y: double): double;
var d: double;
begin
  d:=dist(v1x,v1y)*dist(v2x,v2y);
  if abs(d)<1e-6 then result:=-1
  else result:=(v1x*v2x+v1y*v2y)/d;
end;

procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
var vx,vy: double;
begin
  vx:=px+tx;
  vy:=py+ty;
  rx:=vx*cos(teta)-vy*sin(teta);
  ry:=vx*sin(teta)+vy*cos(teta);
end;

procedure RotateAndTranslate(var rx,ry: double; px,py,tx,ty,teta: double);
var vx,vy: double;
begin
  vx:=px*cos(teta)-py*sin(teta);
  vy:=px*sin(teta)+py*cos(teta);
  rx:=vx+tx;
  ry:=vy+ty;
end;

procedure RotateAroundPoint(var rx,ry: double; px,py,cx,cy,teta: double);
var vx,vy: double;
begin
  vx:=px-cx;
  vy:=py-cy;
  rx:=vx*cos(teta)-vy*sin(teta);
  ry:=vx*sin(teta)+vy*cos(teta);
  rx:=rx+cx;
  ry:=ry+cy;
end;


function NormalizeAngle(ang: double): double;
var a: double;
begin
  a:=FMod(ang+Pi,2*Pi);
  if a<0 then result:=a+Pi
  else result:=a-Pi;
end;


procedure RotateCov( const incov_x,incov_y,incov_xy: double; out cov_x,cov_y, cov_xy: double; teta: double);
var ce,se,t1,t3,t5,t6: double;
begin              // teta=0
  ce:=cos(teta);   // 1
  se:=sin(teta);   // 0
  t1:= ce*ce;      // 1
  t3:= ce*se;      // 0
  t5:= 2.0*t3*incov_xy;  // 0
  t6:= se*se;            // 0
  cov_xy:= t3*incov_x-t6*incov_xy+t1*incov_xy-t3*incov_y;
  cov_x:= t1*incov_x+t5+t6*incov_y;
  cov_y:= t6*incov_x-t5+t1*incov_y;
end;


function IncWrap(var v: integer; size: integer; step:integer=1): integer;
begin
  inc(v,step);
  if v>=size then v:=v-size;
  Result:=v;
end;

function DecWrap(var v: integer; size: integer; step:integer=1): integer;
begin
  dec(v,step);
  if v<0 then v:=v+size;
  Result:=v;
end;

procedure SwapInts(var v1, v2: integer);
var t: integer;
begin
  t:=v1;
  v1:=v2;
  v2:=t;
end;

function Middle(a,b,c: integer): integer;
begin
  if a>b then SwapInts(a,b);
  if b>c then SwapInts(b,c);
  if a>b then SwapInts(a,b);
  result:=b;
end;

function LinInterp(x0, x1, y0, y1, x: double): double;
begin
  result := y0 + (x - x0) * (y1 - y0) / (x1 - x0);
end;


{function BiLinInterp(Surf: matrix; xmin, xmax, ymin, ymax, x,y: double): double;
var dx, dy: double;
    nx, ny, ix, iy: integer;
    A, B, C, D: double;
    yt0, yt1: double;
begin
  result := 0;
  nx := MNumRows(Surf);
  ny := MNumCols(Surf);
  if (nx = 0) or (ny = 0) then exit;
  if ((xmax - xmin) = 0) or ((ymax - ymin) = 0) then exit;

  dx := (nx - 1) / (xmax - xmin);
  dy := (ny - 1) / (ymax - ymin);

  ix := floor((x - xmin) * dx);
  iy := floor((y - ymin) * dy);

  A := Mgetv(Surf, iy, ix);
  B := Mgetv(Surf, iy, ix+1);
  C := Mgetv(Surf, iy+1, ix);
  D := Mgetv(Surf, iy+1, ix+1);

  yt0 := LinInterp(ix * dx, (ix + 1) * dx, A, B, x);
  yt1 := LinInterp(ix * dx, (ix + 1) * dx, C, D, x);
  result := LinInterp(iy * dy, (iy + 1) * dy, yt0, yt1, y);

end;
}
end.




