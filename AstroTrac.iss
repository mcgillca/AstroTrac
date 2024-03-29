; Script generated by the Inno Setup Script Wizard.
; SEE THE DOCUMENTATION FOR DETAILS ON CREATING INNO SETUP SCRIPT FILES!

#define MyAppName "AstroTrac X2 PlugIn"
#define MyAppVersion "1.00"
#define MyAppPublisher "RTI-Zone"
#define MyAppURL "https://rti-zone.org"

[Setup]
; NOTE: The value of AppId uniquely identifies this application.
; Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{75D76F1E-19D9-4825-95EB-28830C840A84}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
;AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={code:TSXInstallDir}\Resources\Common
DefaultGroupName={#MyAppName}

; Need to customise these
; First is where you want the installer to end up
OutputDir=installer
; Next is the name of the installer
OutputBaseFilename=AstroTrac_X2_Installer
; Final one is the icon you would like on the installer. Comment out if not needed.
SetupIconFile=rti_zone_logo.ico
Compression=lzma
SolidCompression=yes
; We don't want users to be able to select the drectory since read by TSXInstallDir below
DisableDirPage=yes
; Uncomment this if you don't want an uninstaller.
;Uninstallable=no
CloseApplications=yes
DirExistsWarning=no

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Dirs]
Name: "{app}\Plugins\MountPlugIns";
Name: "{app}\Plugins64\MountPlugIns";

[Files]
Source: "mountlist AstroTrac.txt";                      DestDir: "{app}\Miscellaneous Files"; Flags: ignoreversion
Source: "mountlist AstroTrac.txt";                      DestDir: "{app}\Miscellaneous Files"; Flags: ignoreversion; DestName: "mountlist64 AstroTrac.txt"
; 32 bits
Source: "libAstroTrac\Win32\Release\libAstroTrac.dll";  DestDir: "{app}\Plugins\MountPlugIns"; Flags: ignoreversion
Source: "AstroTrac.ui";                                 DestDir: "{app}\Plugins\MountPlugIns"; Flags: ignoreversion
Source: "AstroTrac360.png";                             DestDir: "{app}\Plugins\MountPlugIns"; Flags: ignoreversion
; 64 bits
Source: "libAstroTrac\x64\Release\libAstroTrac.dll";    DestDir: "{app}\Plugins64\MountPlugIns"; Flags: ignoreversion; Check: DirExists(ExpandConstant('{app}\Plugins64\MountPlugIns'))
Source: "AstroTrac.ui";                                 DestDir: "{app}\Plugins64\MountPlugIns"; Flags: ignoreversion; Check: DirExists(ExpandConstant('{app}\Plugins64\MountPlugIns'))
Source: "AstroTrac360.png";                             DestDir: "{app}\Plugins64\MountPlugIns"; Flags: ignoreversion; Check: DirExists(ExpandConstant('{app}\Plugins64\MountPlugIns'))

[Code]
{* Below are functions to read TheSkyXInstallPath.txt and confirm that the directory does exist
   This is then used in the DefaultDirName above
   *}
function FindFile(RootPath: string; FileName: string): string;
 var
  FindRec: TFindRec;
  FilePath: string;
begin
  { Log(Format('Searching %s for %s', [RootPath, FileName])); }
  if FindFirst(RootPath + '\*', FindRec) then
  begin
    try
      repeat
        if (FindRec.Name <> '.') and (FindRec.Name <> '..') then
        begin
          FilePath := RootPath + '\' + FindRec.Name;
          if FindRec.Attributes and FILE_ATTRIBUTE_DIRECTORY <> 0 then
          begin
            Result := FindFile(FilePath, FileName);
            if Result <> '' then Exit;
          end
            else
          if CompareText(FindRec.Name, FileName) = 0 then
          begin
            Log(Format('Found %s', [FilePath]));
            Result := FilePath;
            Exit;
          end;
        end;
      until not FindNext(FindRec);
    finally
      FindClose(FindRec);
    end;
  end
    else
  begin
    Log(Format('Failed to list %s', [RootPath]));
  end;
end;


function TSXInstallDir(Param: String) : String;
 var
  TheSkyXInstallPath: String;
  Location: String;
  LoadResult: Boolean;
begin
   TheSkyXInstallPath := FindFile(ExpandConstant('{userdocs}') + '\Software Bisque', 'TheSkyXInstallPath.txt');
  { Check that could open the file}
  if Length(TheSkyXInstallPath)=0 then
    RaiseException('Unable to find the installation path for The Sky X :' + TheSkyXInstallPath);
  LoadResult := LoadStringFromFile(TheSkyXInstallPath, Location)
  {Check that the file exists}
  if not DirExists(Location) then
    RaiseException('The SkyX installation directory ' + Location + ' does not exist');

  Result := Location;
end;

