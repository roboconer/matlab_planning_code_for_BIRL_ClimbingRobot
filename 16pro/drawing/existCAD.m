function existCAD()

if ~exist('BandClamp','var') || ~exist('GModule','var') || ~exist('HollowPart','var') ||...
        ~exist('IModule','var') || ~exist('TModuleP1','var') || ~exist('TModuleP2','var')   % 如果不存在，需要重新读入
    global BandClamp GModule HollowPart IModule TModuleP1 TModuleP2;
    [BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2] = readAllParts();
end

end