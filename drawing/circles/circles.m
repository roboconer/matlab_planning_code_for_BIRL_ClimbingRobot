function varargout = circles(varargin)
%CIRCLES Plot multiple circles as a single patch.
%   CIRCLES(X,Y,R) adds a circle to the current figure, centered at (X,Y)
%   and with radius R. If the inputs are scalars, one circle is added. If
%   the inputs are vectors of length M, then M circles are added.
%
%   H = CIRCLES(___) returns the graphics object handle to the patch
%   containing the circles.
%
%   [___] = CIRCLES(___,Name,Value) uses additional parameter name-value
%   pairs. Valid parameters include:
%
%       'Colormap'      Colormap, specified either as a string for one of
%                       the built-in colormaps (e.g. 'parula', 'jet',
%                       'gray'), or an Nx3 array of RGB triplets, where N
%                       is the number of unique colors. This parameter only
%                       matters if 'EdgeColor' or 'FaceColor' is set to
%                       'shuffle'.
%
%                       Default: 'parula'
%
%       'EdgeColor'     Color of circle edges, expressed as an RGB triplet
%                       (one color for all circles), an Mx3 array of RGB 
%                       triplets (one color for each circle), a color 
%                       string in short (e.g. 'k') or long (e.g. 'black') 
%                       form, 'shuffle' (to randomize the colors in 
%                       'Colormap'), or 'none'.
%
%                       Default: 'k'
%
%       'FaceAlpha'     Transparency of circles, expressed as a numeric
%                       scalar in the range [0,1] or a vector of M alpha
%                       values, one for each circle.
%
%                       Default: 1
%
%       'FaceColor'     Color of circle faces, expressed as an RGB triplet
%                       (one color for all circles), an Mx3 array of RGB 
%                       triplets (one color for each circle), a color 
%                       string in short (e.g. 'k') or long (e.g. 'black') 
%                       form, 'shuffle' (to randomize the colors in 
%                       'Colormap'), or 'none'.
%
%                       Default: 'none'
%
%       'LineStyle'     Line style, specified as one of the following
%                       strings: {'-','--',':','-.','none'}.
%
%                       Default: '-'
%
%       'LineWidth'     Line width, specified as a positive numeric scalar
%                       in points.
%
%                       Default: 1.5
%
%       'Resolution'    Angular resolution of each circle, in degrees, 
%                       specified as a numeric scalar. Value must be in the
%                       range (0,180].
%
%                       Default: 10
%
%   Notes:
%   1) Colors expressed as RGB triplets must be in the range [0,1].
%
%   2) Multiple colors cannot be used for both edges and faces. If multiple
%   colors are specified for both, the face colors take precendence. For 
%   example, if you set 'EdgeColor' to parula(M) and 'FaceColor' to jet(M),
%   then both the edges and faces will be set according to jet(M).
%
%   See also PATCH, RECTANGLES, VISCIRCLES.

% Copyright 2016 Matthew R. Eicholtz

% Default parameter values
default = struct(...
    'Colormap','parula',...
    'EdgeColor','k',...
    'FaceAlpha',1,...
    'FaceColor','none',...
    'LineStyle','-',...
    'LineWidth',1.5,...
    'Resolution',10);

% Parse inputs
[x,y,r,theta,params] = parseinputs(default,varargin{:});

% Setup circle x-y data
X = bsxfun(@times,r(:)',cos(theta));
X = bsxfun(@plus,X,x(:)');
X = X(:);

Y = bsxfun(@times,r(:)',sin(theta));
Y = bsxfun(@plus,Y,y(:)');
Y = Y(:);

f = reshape(1:length(X),[],length(x))'; %faces
v = [X,Y]; %vertices

% Draw circle(s) as patch
holdon = ishold;
hold on;
h = patch(...
    'Faces',f,...
    'Vertices',v,...
    params{:});
if ~holdon; hold off; end

% Return output, if requested
varargout = {h};

end

%% Helper functions
function varargout = parseinputs(default,varargin)
    p = inputParser;
    
    p.addRequired('x',@(x) validateattributes(x,...
        {'numeric'},{'2d','real','finite','nonempty','nonsparse'}));
    p.addRequired('y',@(x) validateattributes(x,...
        {'numeric'},{'2d','real','finite','nonempty','nonsparse'}));
    p.addRequired('r',@(x) validateattributes(x,...
        {'numeric'},{'2d','real','finite','nonempty','nonsparse','>',0}));
    p.addParameter('Colormap',default.Colormap,@validateColormap);
    p.addParameter('EdgeColor',default.EdgeColor,@validateEdgeColor);
    p.addParameter('FaceAlpha',default.FaceAlpha,@validateFaceAlpha);
    p.addParameter('FaceColor',default.FaceColor,@validateFaceColor);
    p.addParameter('LineStyle',default.LineStyle,@validateLineStyle);
    p.addParameter('LineWidth',default.LineWidth,@validateLineWidth);
    p.addParameter('Resolution',default.Resolution,@(x) validateattributes(x,...
        {'numeric'},{'scalar','real','finitie','nonempty','nonsparse','>',0,'<=',180}));
    
    p.parse(varargin{:});
    
    x = p.Results.x;
    y = p.Results.y;
    r = p.Results.r;
    cmap = p.Results.Colormap;
    edgecolor = p.Results.EdgeColor;
    facealpha = p.Results.FaceAlpha;
    facecolor = p.Results.FaceColor;
    linestyle = p.Results.LineStyle;
    linewidth = p.Results.LineWidth;
    resolution = p.Results.Resolution;
    
    assert(isequal(size(x),size(y),size(r)),'The inputs X, Y, and R must be vectors of the same length');
    
    N = length(x);
    theta = (0:resolution:360)'*pi/180;
    n = length(theta); %number of points per circle
    
    if ischar(cmap)
        cmap = feval(cmap,N);
    end
    
    if strcmp(edgecolor,'shuffle')
        edgecolor = 'flat';
        ind = randsample(size(cmap,1),N,size(cmap,1)<N);
        cmap = cmap(ind,:);
        cmap = kron(cmap,ones(n,1));
    end
    if size(edgecolor,1)>1
        assert(isequal(N,size(edgecolor,1)),'Number of edge colors must be the same as the number of rectangles.');
        cmap = edgecolor;
        cmap = kron(cmap,ones(n,1));
        edgecolor = 'flat';
    end
    
    if strcmp(facecolor,'shuffle')
        facecolor = 'flat';
        ind = randsample(size(cmap,1),N,size(cmap,1)<N);
        cmap = cmap(ind,:);
        cmap = kron(cmap,ones(n,1));
    end
    if size(facecolor,1)>1
        assert(isequal(N,size(facecolor,1)),'Number of face colors must be the same as the number of rectangles.');
        cmap = facecolor;
        cmap = kron(cmap,ones(n,1));
        facecolor = 'flat';
    end
    
    if length(facealpha)>1
        assert(isequal(N,length(facealpha)),'FaceAlpha must be a scalar or an Mx1 vector of alpha values, one for each circle.');
        amap = facealpha(:);
        facealpha = 'flat';
    else
        amap = 1;
    end
    
    cmap = permute(cmap,[1 3 2]);
    
    params = {...
        'FaceColor',facecolor,...
        'FaceAlpha',facealpha,...
        'EdgeColor',edgecolor,...
        'LineStyle',linestyle,...
        'Linewidth',linewidth,...
        'CData',cmap,...
        'CDataMapping','direct',...
        'FaceVertexAlphaData',amap};
    
    varargout = {x,y,r,theta,params};
end

function tf = validateColormap(x)
    tf = (ismatrix(x) && size(x,2)==3 && all(x(:)>=0) && all(x(:)<=1)) || ... %array of RGB triplets
        ismember(x,{'parula','jet','hsv','hot','cool','spring','summer',... %colormap string
        'autumn','winter','gray','bone','copper','pink','lines','colorcube',...
        'prism','flag','white'});
end
function tf = validateEdgeColor(x)
    tf = ismatrix(x) && size(x,2)==3 && all(x(:)>=0) && all(x(:)<=1) || ... %array of RGB triplets
        ischar(x) && ~isempty(str2rgb(x)) || ... %color string
        ismember(x,{'shuffle','none'}); %other valid strings
end
function tf = validateFaceAlpha(x)
    tf = (isscalar(x) && x>=0 && x<=1) || ... %scalar in range [0,1]
        (isvector(x) && all(x(:)>=0) && all(x(:)<=1)); %vector in range [0,1], one for each circle
end
function tf = validateFaceColor(x)
    tf = ismatrix(x) && size(x,2)==3 && all(x(:)>=0) && all(x(:)<=1) || ... %array of RGB triplets
        ischar(x) && ~isempty(str2rgb(x)) || ... %color string
        ismember(x,{'shuffle','none'}); %other valid strings
end
function tf = validateLineStyle(x)
    tf = ismember(x,{'-','--',':','-.','none'});
end
function validateLineWidth(x)
    validateattributes(x,{'numeric'},{'scalar','real','finite','>',0});
end

function rgb = str2rgb(str)
    rgb = dec2bin(rem(find(strcmpi(strsplit('k b g c r m y w black blue green cyan red magenta yellow white'),str))-1,8),3)-'0';
end

