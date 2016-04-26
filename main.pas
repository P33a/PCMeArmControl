{ PCMEArmControl v1.1

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

unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, SynEdit, lNetComponents, SdpoSerial, SdpoJoystick,
  Forms, Controls,Graphics, Dialogs, StdCtrls, IniPropStorage, ExtCtrls, ComCtrls, Math,
  lclintf, Grids, lNet;

const
  NUM_JOINTS = 4;

type
  TServoControl =  record
    angle, read_pulse: integer;
    read_angle: double;
    max, zero, min: integer;
    speed: byte;

    // Calibration data
    angle0, angle1, pulse0, pulse1: integer;

    // Calibration Interface
    EditMin, EditZero, EditMax, EditValue, EditSpeed: TEdit;
    SB: TScrollBar;
  end;

  TServosControl = array[0..NUM_JOINTS - 1] of TServoControl;

  TJointAngles = array [0..NUM_JOINTS - 1] of double;

  { TFMain }

  TFMain = class(TForm)
    BMovXSet: TButton;
    BGripSet: TButton;
    BOpenComPort: TButton;
    BCloseComPort: TButton;
    BMovYSet: TButton;
    BLoadJointsList: TButton;
    BSerialSendRaw: TButton;
    BReset: TButton;
    BSerialSendDecimal: TButton;
    BSetConfig: TButton;
    BMemoClear: TButton;
    BRotZSet: TButton;
    BCalibrate: TButton;
    BSaveServosCalibration: TButton;
    BLoadServosCalibration: TButton;
    BPlayJoints: TButton;
    BSaveJointsList: TButton;
    CBLANSendAngles: TCheckBox;
    CBRawDebug: TCheckBox;
    CBJoystick: TCheckBox;
    CBSerialSendJoy: TCheckBox;
    CBLANSendJoy: TCheckBox;
    CBSerialSendAngles: TCheckBox;
    CBPlayContinous: TCheckBox;
    EditShowSeq: TEdit;
    EditGripMax: TEdit;
    EditGripMin: TEdit;
    EditGripSpeed: TEdit;
    EditGripValue: TEdit;
    EditGripZero: TEdit;
    EditMovYMax: TEdit;
    EditMovXMax: TEdit;
    EditMovYMin: TEdit;
    EditMovXMin: TEdit;
    EditMovXSpeed: TEdit;
    EditMovXValue: TEdit;
    EditMovXZero: TEdit;
    EditRotZSpeed: TEdit;
    EditMovYSpeed: TEdit;
    EditRotZValue: TEdit;
    EditRotZMin: TEdit;
    EditDebug: TEdit;
    EditDecimalData: TEdit;
    EditChannel: TEdit;
    EditMovYValue: TEdit;
    EditRotZZero: TEdit;
    EditRotZMax: TEdit;
    EditMovYZero: TEdit;
    EditShowSeqLen: TEdit;
    EditTicksToRads: TEdit;
    EditWheelDiameter: TEdit;
    EditWheelDist: TEdit;
    Joystick: TSdpoJoystick;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    SGJointAngles: TStringGrid;
    SGReadJointAngles: TStringGrid;
    TabJoints: TTabSheet;
    UDP: TLUDPComponent;
    SGCalibration: TStringGrid;
    PageControl: TPageControl;
    EditRawData: TEdit;
    EditComPort: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    SBMovX: TScrollBar;
    SBGrip: TScrollBar;
    SBRotZ: TScrollBar;
    SBMovY: TScrollBar;
    Serial: TSdpoSerial;
    SGJoystickAxis: TStringGrid;
    SGJoystickButtons: TStringGrid;
    SynEditDebug: TSynEdit;
    TabDebug: TTabSheet;
    TabConfig: TTabSheet;
    TabCalib: TTabSheet;
    TabJoystick: TTabSheet;
    TimerJoy: TTimer;
    procedure BCalibrateClick(Sender: TObject);
    procedure BCloseComPortClick(Sender: TObject);
    procedure BLoadJointsListClick(Sender: TObject);
    procedure BLoadServosCalibrationClick(Sender: TObject);
    procedure BMemoClearClick(Sender: TObject);
    procedure BOpenComPortClick(Sender: TObject);
    procedure BPlayJointsClick(Sender: TObject);
    procedure BResetClick(Sender: TObject);
    procedure BSaveJointsListClick(Sender: TObject);
    procedure BSaveServosCalibrationClick(Sender: TObject);
    procedure BServoSetClick(Sender: TObject);
    procedure BSerialSendDecimalClick(Sender: TObject);
    procedure BSerialSendRawClick(Sender: TObject);
    procedure BSetConfigClick(Sender: TObject);
    procedure CBJoystickClick(Sender: TObject);
    procedure EditServoDblClick(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure SBServoChange(Sender: TObject);
    procedure SerialRxData(Sender: TObject);
    procedure TimerJoyTimer(Sender: TObject);
    procedure UDPError(const msg: string; aSocket: TLSocket);
    procedure UDPReceive(aSocket: TLSocket);
  private
    seq, seqLen: integer;
    start_time: DWord;

    procedure Debugln(s: string);
    procedure playJoints;
    procedure processFrame(channel: char; value: integer; source: integer);
    procedure ReceiveData(s: string);
    procedure SendChannel(channel: char; value: integer);
    procedure SetComState(newState: boolean);
    { private declarations }

  public
    serialData: string;

    channel: char;
    nfile, frame, frameSource: integer;
    frameData: string;
    sample: integer;
    discount: integer;

    sline: string;

    stateEnterTime: TTime;
    cis: integer;

    Servos: TServosControl;

    SerialAnglesReq: TJointAngles;
    RefJ, ReqRefJ: TJointAngles;

    function tis: TTime;
    procedure Debug(s: string);
  end;

var
  FMain: TFMain;

//procedure AsyncSendMessage(c: char; val: integer);
procedure SendMessage(c: char; val: integer);
procedure SendRaw(s: string);


implementation


{$R *.lfm}

procedure SendMessage(c: char; val: integer);
begin
  FMain.Serial.WriteData(c + IntToHex(dword(Val) and $FFFFFFFF, 8));
end;

procedure SendRaw(s: string);
begin
  FMain.Serial.WriteData(s);
end;

function map(x, in_min, in_max, out_min, out_max: integer): integer;
begin
  result := (x - in_min) * (out_max - out_min) div (in_max - in_min) + out_min;
end;


function map(x, in_min, in_max, out_min, out_max: double): double;
begin
  result := (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
end;


{ TFMain }

procedure TFMain.SetComState(newState: boolean);
var comColor: TColor;
begin
  try
  if newState then begin
    Serial.Device := EditComPort.Text;
    Serial.Open;
  end else begin
    Serial.Close;
  end;
  finally
    if Serial.Active then comColor := clGreen
    else comColor := clRed;
    EditComPort.Color := comColor;
  end;
end;



procedure TFMain.Debug(s: string);
begin
  //MemoDebug.Lines.Add(s);
  //SynEditDebug.Text := SynEditDebug.Text + s;

  SynEditDebug.CaretY := maxint;
  SynEditDebug.CaretX := maxint;
  SynEditDebug.InsertTextAtCaret(s);
  while SynEditDebug.Lines.Count > 100 do begin
    SynEditDebug.Lines.Delete(0);
  end;

  //SynEditDebug.EnsureCursorPosVisible;
end;

procedure TFMain.Debugln(s: string);
begin
  Debug(s + chr(10));
end;


procedure TFMain.SendChannel(channel: char; value: integer);
begin
  if Serial.Active then begin
    Serial.WriteData(channel + IntToHex(value, 8));
  end;
end;

procedure TFMain.BOpenComPortClick(Sender: TObject);
begin
  SetComState(true);
end;

// Start sequence
procedure TFMain.BPlayJointsClick(Sender: TObject);
begin
  seqLen := 0;
  while SGJointAngles.Cells[1 + seqLen, 0] <> '' do begin
    inc(seqLen);
  end;
  if seqLen > 0 then begin
    seq := 0;
    start_time := GetTickCount();
  end;
  EditShowSeqLen.Text := IntToStr(seqLen);
end;

procedure TFMain.BResetClick(Sender: TObject);
begin
  if Serial.Active then begin
    Serial.SetDTR(false);
    Serial.SetDTR(true);
  end;
end;

procedure TFMain.BSaveJointsListClick(Sender: TObject);
begin
  SGJointAngles.SaveToFile('jointangles.xml');
end;

procedure TFMain.BSaveServosCalibrationClick(Sender: TObject);
begin
  SGCalibration.SaveToFile('calibration.xml');
end;

//------------------------------------------------------------------------------
// Servos interface

procedure TFMain.BServoSetClick(Sender: TObject);
var i: integer;
begin
  i := (Sender as TButton).Tag;
  Servos[i].min := StrToInt(Servos[i].EditMin.Text);
  Servos[i].zero := StrToInt(Servos[i].EditZero.Text);
  Servos[i].max := StrToInt(Servos[i].EditMax.Text);
  Servos[i].angle := StrToInt(Servos[i].EditValue.Text);

  Servos[i].speed := StrToInt(Servos[i].EditSpeed.Text);
  //SendMessage('w', (i shl 24) or Servos[i].speed);

  Servos[i].SB.SetParams(Servos[i].angle, Servos[i].min, Servos[i].max);
end;

procedure TFMain.SBServoChange(Sender: TObject);
var i: integer;
begin
  i := (Sender as TScrollBar).Tag;
  Servos[i].angle := (Sender as TScrollBar).Position;
  Servos[i].EditValue.Text := IntToStr(Servos[i].angle) ;
end;


procedure TFMain.EditServoDblClick(Sender: TObject);
var i: integer;
begin
  i := (Sender as TEdit).Tag;
  Servos[i].angle := StrToInt((Sender as TEdit).Text);
  Servos[i].SB.Position := Servos[i].angle;
end;


procedure TFMain.BSerialSendDecimalClick(Sender: TObject);
var c: char;
begin
  if EditChannel.Text <> '' then c := EditChannel.Text[1];
  SendMessage(c, StrToInt(EditDecimalData.Text));
end;

procedure TFMain.BSerialSendRawClick(Sender: TObject);
begin
  Serial.WriteData(EditRawData.Text);
end;

procedure TFMain.BMemoClearClick(Sender: TObject);
begin
  SynEditDebug.Lines.Clear;
  sample := 0;
end;


procedure TFMain.BSetConfigClick(Sender: TObject);
begin
  //TicsToRads := strtofloat(EditTicksToRads.Text);
end;

procedure TFMain.CBJoystickClick(Sender: TObject);
begin
  try
    if CBJoystick.Checked then begin
      //Joystick.DeviceLin := EditJoyDevice.Text;
      Joystick.Active := true;
    end else begin
      Joystick.Active := false;
    end;
  finally
    CBJoystick.Checked := Joystick.Active;
  end;
end;





procedure TFMain.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  IniPropStorage.WriteBoolean('COM_port_state', Serial.Active);
  IniPropStorage.Save;
  Serial.Active := false;
end;

procedure TFMain.FormCreate(Sender: TObject);
begin
  frameSource := 0;
  frame := -1;
  discount := 0;
  seq := -1;

  with Servos[0] do begin
    EditMin := EditRotZMin;
    EditZero := EditRotZZero;
    EditMax := EditRotZMax;
    EditValue := EditRotZValue;
    EditSpeed := EditRotZSpeed;
    SB := SBRotZ;
  end;

  with Servos[1] do begin
    EditMin := EditMovYMin;
    EditZero := EditMovYZero;
    EditMax := EditMovYMax;
    EditValue := EditMovYValue;
    EditSpeed := EditMovYSpeed;
    SB := SBMovY;
  end;

  with Servos[2] do begin
    EditMin := EditMovXMin;
    EditZero := EditMovXZero;
    EditMax := EditMovXMax;
    EditValue := EditMovXValue;
    EditSpeed := EditMovXSpeed;
    SB := SBMovX;
  end;

  with Servos[3] do begin
    EditMin := EditGripMin;
    EditZero := EditGripZero;
    EditMax := EditGripMax;
    EditValue := EditGripValue;
    EditSpeed := EditGripSpeed;
    SB := SBGrip;
  end;

end;

procedure TFMain.FormShow(Sender: TObject);
var openSerial: boolean;
begin
  BLoadServosCalibration.Click;
  BCalibrate.Click;

  BMemoClear.Click;
  BRotZSet.Click;
  BMovYSet.Click;
  BMovXSet.Click;
  BGripSet.Click;

  BLoadJointsList.Click;

  openSerial := IniPropStorage.ReadBoolean('COM_port_state', true);
  SetComState(openSerial);

  if CBLANSendJoy.Checked or CBLANSendAngles.Checked then begin
    UDP.Listen(9909);
    //UDP.Connect('127.0.0.1', 9808);
  end;
end;

function TFMain.tis: TTime;
begin
  result := now() - stateEnterTime;
end;


function isHexDigit(c: char): boolean;
begin
  result := c in ['0'..'9', 'A'..'F'];
end;

procedure TFMain.processFrame(channel: char; value: integer; source: integer);
var s: string;
    i: integer;
begin
  //MemoDebug.Text := MemoDebug.Text + channel;
  if channel = 'i' then begin
  end else if channel = 'r' then begin
    // if the arduino was reset it must receive new servo values
    BRotZSet.Click;
    BMovYSet.Click;
    BMovXSet.Click;
    BGripSet.Click;
    // SendMessage('w', );
  end else if channel = 's' then begin
    // Reading of the current angle
    i := (value shr 24) and $FF;
    if i < NUM_JOINTS then begin
      Servos[i].read_pulse := (value and $FFFF);
      Servos[i].read_angle := map(double(Servos[i].read_pulse),
                                  Servos[i].pulse0, Servos[i].pulse1,
                                  Servos[i].angle0, Servos[i].angle1);

      SGReadJointAngles.Cells[1, 1 + i] := IntToStr(Servos[i].read_pulse);
      SGReadJointAngles.Cells[2, 1 + i] := format('%4.1f', [Servos[i].read_angle]);
    end;
  end else if channel = 'q' then begin
    i := (value shr 24) and $FF;
    if i < NUM_JOINTS then begin
      SerialAnglesReq[i] := ((value and $FFFF) - 32767) / 10;
    end;
  end else if channel = 'z' then begin
    SBRotZ.Position := value;
  end else if channel = 'x' then begin
    SBMovX.Position := value;
  end else if channel = 'y' then begin
    SBMovY.Position := value;
  end else if channel = 'w' then begin
    SBGrip.Position := value;

  end else if channel in ['s', 't'] then begin
    i := 1 + ord(channel) - ord('r');

  end;
end;


procedure TFMain.ReceiveData(s: string);
var //b: byte;
    c: char;
    value: integer;
begin
  if s = '' then exit;
  serialData := serialData + s;

  if CBRawDebug.Checked then begin
    Debug(s);
  end;

  while Length(serialData) > 0 do begin
    c := serialData[1];
    serialData := copy(serialData, 2, maxint);
    if frame = -1 then begin

      if c = '*' then frameSource := 0
      else if c = '+' then frameSource := 1
      else if c = '-' then frameSource := 2;

      if (c in ['G'..'Z']) or (c in ['g'..'z']) then begin
        frame := 0;
        channel := c;
        frameData := '';
      end;
    end else begin
      if isHexDigit(c) then begin
        frameData := frameData + c;
        inc(frame);
        if frame = 8 then begin
          value := StrToIntDef('$' + frameData, -1);
          processFrame(channel, value, frameSource);
          frame := -1;
        end;
      end else begin
        frame := -1;
      end;
    end;
  end;
end;

function sat(v, absmax: double): double;
begin
  result := max(min(v, absmax), -absmax);
end;

procedure TFMain.playJoints;
var i: integer;
    dt, speed: double;
    change_time, min_time: DWord;
    joint_err: double;
begin
  if seq >= 0 then begin
    speed := StrToFloat(SGJointAngles.Cells[1 + seq, 4]);
    if speed = 0 then speed := 1;

    dt := StrToFloat(SGJointAngles.Cells[1 + seq, 5]);
    min_time := start_time + round(1000 * dt);
    dt := StrToFloat(SGJointAngles.Cells[1 + seq, 6]);
    change_time := start_time + round(1000 * dt);

    // for all Joints
    for i := 0 to NUM_JOINTS - 1 do begin
      ReqRefJ[i] := StrToInt(SGJointAngles.Cells[1 + seq, i]);
      RefJ[i] := ReqRefJ[i];
    end;

    joint_err := 0;
    for i := 0 to NUM_JOINTS - 2 do begin // Exclude last joint (gripper)
      joint_err := max(joint_err, abs(Servos[i].read_angle - RefJ[i]));
    end;
    EditDebug.Text := inttostr(round(joint_err));

    // Test if max time has passed or angle was reached after min time
    if (GetTickCount() > change_time) or ((GetTickCount() > min_time) and (joint_err < 2)) then begin
      inc(seq);
      if seq >= seqLen then begin
        if not CBPlayContinous.Checked then begin
          seq := -1;
        end else begin
          seq := 0;
          start_time := GetTickCount();
        end;
      end else begin
        start_time := GetTickCount();
      end;
    end;

    for i := 0 to NUM_JOINTS - 1 do begin
      Servos[i].angle := round(RefJ[i]);
      Servos[i].speed := round(speed);
    end;
  end;
  EditShowSeq.Text := IntToStr(seq);

end;

procedure TFMain.SerialRxData(Sender: TObject);
//var s: string;
begin
 // exit;
  //s := Serial.ReadData;
  //if s <> '' then ReceiveData(s);
  //if CBRawDebug.Checked then Debugln(s);
end;

procedure TFMain.TimerJoyTimer(Sender: TObject);
var i, joybs, b: integer;
    axisNeg: array [0..3] of integer = (1, -1, -1, 1);
    axisChannel: array [0..3] of char = ('X', 'Y', 'Z', 'W');
    axisValue: array [0..3] of integer;
    mess, s: string;
begin

  // Read and process serial port data (connection to Arduino)
  s := '';
  if Serial.Active then s := Serial.ReadData;
  if s <> '' then ReceiveData(s);
  if CBRawDebug.Checked then Debugln(s);

  // Read ans process UDP Packet
  mess := '';
  while true do begin
    if UDP.Connected then begin
      UDP.GetMessage(mess);
    end else begin
      if CBLANSendJoy.Checked or CBLANSendAngles.Checked then begin
        inc(discount);
        if discount > 10 then begin
          //UDP.Disconnect(true);
          UDP.Disconnect();
          UDP.Listen(9909);
          discount := 0;
        end;
      end;
    end;
    if mess = '' then break;
    if CBRawDebug.Checked then Debugln(mess);
  end;

  playJoints();

  if CBSerialSendAngles.Checked then begin
    for i := 0 to NUM_JOINTS - 1 do begin
      SendMessage('S', (i shl 24) or
                       (Servos[i].speed shl 16) or
                       map(Servos[i].angle,
                           Servos[i].angle0, Servos[i].angle1,
                           Servos[i].pulse0, Servos[i].pulse1));
    end;
  end;

  if CBLANSendAngles.Checked then begin
    mess := 'SA';
    for i := 0 to NUM_JOINTS - 1 do begin
      mess := mess + IntToHex(Servos[i].angle, 8);
    end;
    if UDP.Connected then
      UDP.SendMessage(mess, '127.0.0.1:9808');
  end;


  if not Joystick.Active then exit;

  // Read Axis from Joystick and send over serial port (to Arduino) if requested
  for i := 0 to 3 do begin
    axisValue[i] := min(32767, max(-32767, axisNeg[i] * (Joystick.Axis[i] - 32767)));
    SGJoystickAxis.Cells[1, i + 1] := IntToStr(axisValue[i]);
    if CBSerialSendJoy.Checked then begin
      SendChannel(axisChannel[i], axisValue[i]);
    end;
  end;

  // Read Buttons from Joystick and send over serial port (to Arduino) if requested
  joybs := 0;
  for i := 0 to 11 do begin
    b := Joystick.Buttons[i];
    SGJoystickButtons.Cells[1, i + 1] := IntToStr(b);
    joybs := joybs or (b shl i);
  end;
  if CBSerialSendJoy.Checked then begin
    SendChannel('O', joybs);
  end;

  // send Joystic buttons and axis over UDP connection (to SimTwo)
  if CBLANSendJoy.Checked then begin
    mess := 'JY' + IntToHex(joybs, 8);
    for i := 0 to 3 do begin
      mess := mess + IntToHex(axisValue[i], 8);
    end;
    if UDP.Connected then
      UDP.SendMessage(mess, '127.0.0.1:9808');
  end;

  //EditDebug.Text := mess//IntToHex(joybs, 4);
end;

procedure TFMain.UDPError(const msg: string; aSocket: TLSocket);
begin
  Debugln(msg);
end;

procedure TFMain.UDPReceive(aSocket: TLSocket);
//var data: string;
begin
  //data := '';
  //if UDP.Connected then UDP.GetMessage(data);
  //Debugln(data);
  //aSocket.GetMessage(data);
  //SetLength(data, 65535);
  //UDP.Get((@data[1])^, 4096);
end;

procedure TFMain.BCloseComPortClick(Sender: TObject);
begin
  SetComState(false);
end;

procedure TFMain.BLoadJointsListClick(Sender: TObject);
begin
  if FileExists('jointangles.xml') then
    SGJointAngles.LoadFromFile('jointangles.xml');
end;

procedure TFMain.BLoadServosCalibrationClick(Sender: TObject);
begin
  if FileExists('calibration.xml') then
    SGCalibration.LoadFromFile('calibration.xml');
end;

procedure TFMain.BCalibrateClick(Sender: TObject);
var i: integer;
begin
//  SGCalibration.SaveToFile();
  for i := 0 to NUM_JOINTS - 1 do begin
    with Servos[i] do begin
      angle0 := StrToInt(SGCalibration.Cells[1, 1 + i]);
      angle1 := StrToInt(SGCalibration.Cells[3, 1 + i]);
      pulse0 := StrToInt(SGCalibration.Cells[2, 1 + i]);
      pulse1 := StrToInt(SGCalibration.Cells[4, 1 + i]);
    end;
  end;
end;



end.

