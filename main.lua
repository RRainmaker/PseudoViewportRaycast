-- Credits to OnlyJaycbee on Roblox for the code

--Gilbert-Johnson-Keerthi distance and intersection tests
--Tyler R. Hoyer
--11/20/2014

--May return early if no intersection if found. If it is primed, it will run in amortized constant time (untested).

--If the distance function is used between colliding objects, the program
--may loop a hundred times without finding a result. If this is the case, 
--it will throw an error. The check is omited for speed. If the objects
--might intersect eachother, call the intersection method first.

--Objects must implement the :getFarthestPoint(dir) function which returns the
--farthest point in a given direction.

--Used Roblox's Vector3 userdata. Outside implementations will require a implementation of the methods of
--the Vector3's. :Dot, :Cross, .new, and .magnitude must be defined.

local abs = math.abs
local min = math.min
local huge = math.huge
local origin = Vector3.new()

local function loopRemoved(data, step)
	--We're on the next step
	step = step + 1
	
	--If we have completed the last cycle, stop
	if step > #data then
		return nil
	end
	
	--To be the combination without the value
	local copy = {}
	
	--Copy the data up to the missing value
	for i = 1, step - 1 do
		copy[i] = data[i]
	end
	
	--Copy the data on the other side of the missing value
	for i = step, #data - 1 do
		copy[i] = data[i + 1]
	end
	
	--return the step, combination, and missing value
	return step, copy, data[step]
end

--Finds the vector direction to search for the next point
--in the simplex. 
local function getDir(points, to)
	--Single point, return vector
	if #points == 1 then
		return to - points[1]
		
	--Line, return orthogonal line
	elseif #points == 2 then
		local v1 = points[2] - points[1]
		local v2 = to - points[1]
		return v1:Cross(v2):Cross(v1)
		
	--Triangle, return normal
	else
		local v1 = points[3] - points[1]
		local v2 = points[2] - points[1]
		local v3 = to - points[1]
		local n = v1:Cross(v2)
		return n:Dot(v3) < 0 and -n or n
	end
end

--The function that finds the intersection between two sets
--of points, s1 and s2. s1 and s2 must return the point in
--the set that is furthest in a given direction when called.
--If the start direction sV is specified as the seperation
--vector, the program runs in constant time. (excluding the
--user implemented functions for finding the furthest point).
function Intersection(s1, s2, sV)
	local points = {}

	-- find point 
	local function support(dir)
		local a = s1(dir)
		local b = s2(-dir)
		points[#points + 1] = a - b
		return dir:Dot(a) < dir:Dot(b)
	end
	
	-- find all points forming a simplex
	if support(sV)
		or support(getDir(points, origin))
		or support(getDir(points, origin))
		or support(getDir(points, origin))
	then
		return false
	end

	local step, others, removed = 0, nil,nil
	repeat
		step, others, removed = loopRemoved(points, step)
		local dir = getDir(others, removed)
		if others[1]:Dot(dir) > 0 then
			points = others
			if support(-dir) then
				return false
			end
			step = 0
		end
	until step == 4
	
	return true
end

--Checks if two vectors are equal
local function equals(p1, p2)
	return p1.x == p2.x and p1.y == p2.y and p1.z == p2.z
end

--Gets the mathematical scalar t of the parametrc line defined by
--o + t * v of a point p on the line (the magnitude of the projection).
local function getT(o, v, p)
	return (p - o):Dot(v) / v:Dot(v)
end

--Returns the scalar of the closest point on a line to
--the origin. Note that if the vector is a zero vector then
--it treats it as a point offset instead of a line.
local function lineToOrigin(o, v)
	if equals(v, origin) then
		return o
	end
	local t = getT(o, v, origin)
	if t < 0 then 
		t = 0
	elseif t > 1 then 
		t = 1
	end
	return o + v*t
end

--Convoluted to deal with cases like points in the same place
local function closestPoint(a, b, c)
	--if abc is a line
	if c == nil then
		--get the scalar of the closest point
		local dir = b - a
		local t = getT(a, dir, origin)
		if t < 0 then t = 0
		elseif t > 1 then t = 1
		end
		--and return the point
		return a + dir * t
	end
	
	--Otherwise it is a triangle.
	--Define all the lines of the triangle and the normal
	local vAB, vBC, vCA = b - a, c - b, a - c
	local normal = vAB:Cross(vBC)
	
	--If two points are in the same place then
	if normal.magnitude == 0 then
		
		--Find the closest line between ab and bc to the origin (it cannot be ac)
		local ab = lineToOrigin(a, vAB)
		local bc = lineToOrigin(b, vBC)
		if ab.magnitude < bc.magnitude then
			return ab
		else
			return bc
		end
		
	--The following statements find the line which is closest to the origin
	--by using voroni regions. If it is inside the triangle, it returns the
	--normal of the triangle.
	elseif a:Dot(a + vAB * getT(a, vAB, c) - c) <= 0 then
		return lineToOrigin(a, vAB)
	elseif b:Dot(b + vBC * getT(b, vBC, a) - a) <= 0 then
		return lineToOrigin(b, vBC)
	elseif c:Dot(c + vCA * getT(c, vCA, b) - b) <= 0 then
		return lineToOrigin(c, vCA)
	else
		return -normal * getT(a, normal, origin)
	end
end

--The distance function. Works like the intersect function above. Returns
--the translation vector between the two closest points.
local function distance(s1, s2, sV)
	local function support (dir)
		return s1(dir) - s2(-dir)
	end
	
	--Find the initial three points in the search direction, opposite of the
	--search direction, and in the orthoginal direction between those two 
	--points to the origin.
	local a = support(sV)
	local b = support(-a)
	local c = support(-closestPoint(a, b))
	
	--Setup maximum loops
	local i = 1
	while i < 100 do
		i = i + 1
		
		--Get the closest point on the triangle
		local p = closestPoint(a, b, c)
		
		--If it is the origin, the objects are just touching, 
		--return a zero vector.
		if equals(p, origin) then
			return origin
		end
		
		--Search in the direction from the closest point
		--to the origin for a point. 
		local dir = p.unit
		local d = support(dir)
		local dd = d:Dot(dir)
		local dm = math.min(
			a:Dot(dir),
			b:Dot(dir),
			c:Dot(dir)
		)
		
		--If the new point is farther or equal to the closest 
		--point on the triangle, then we have found the closest 
		--point.
		if dd >= dm then
			--return the point on the minkowski difference as the
			--translation vector between the two closest point.
			return -p
		end
		
		--Otherwise replace the point on the triangle furthest 
		--from the origin with the new point
		local ma, mb, mc = a:Dot(dir), b:Dot(dir), c:Dot(dir)
		if ma > mb then
			if ma > mc then
				a = d
			else
				c = d
			end
		elseif mb > mc then
			b = d
		else
			c = d
		end
	end
	
	--Return an error if no point was found in the maximum 
	--number of iterations
	error 'Unable to find distance, are they intersecting?'
end

--Gilbert-Johnson-Keerthi distance and intersection tests
--Tyler R. Hoyer
--11/20/2014

--May return early if no intersection if found. If it is primed, it will run in amortized constant time (untested).

--If the distance function is used between colliding objects, the program
--may loop a hundred times without finding a result. If this is the case, 
--it will throw an error. The check is omited for speed. If the objects
--might intersect eachother, call the intersection method first.

--Objects must implement the :getFarthestPoint(dir) function which returns the
--farthest point in a given direction.

--Used Roblox's Vector3 userdata. Outside implementations will require a implementation of the methods of
--the Vector3's. :Dot, :Cross, .new, and .magnitude must be defined.

local abs = math.abs
local min = math.min
local huge = math.huge
local origin = Vector3.new()

local function loopRemoved(data, step)
	--We're on the next step
	step = step + 1

	--If we have completed the last cycle, stop
	if step > #data then
		return nil
	end

	--To be the combination without the value
	local copy = {}

	--Copy the data up to the missing value
	for i = 1, step - 1 do
		copy[i] = data[i]
	end

	--Copy the data on the other side of the missing value
	for i = step, #data - 1 do
		copy[i] = data[i + 1]
	end

	--return the step, combination, and missing value
	return step, copy, data[step]
end

--Finds the vector direction to search for the next point
--in the simplex. 
local function getDir(points, to)
	--Single point, return vector
	if #points == 1 then
		return to - points[1]

		--Line, return orthogonal line
	elseif #points == 2 then
		local v1 = points[2] - points[1]
		local v2 = to - points[1]
		return v1:Cross(v2):Cross(v1)

		--Triangle, return normal
	else
		local v1 = points[3] - points[1]
		local v2 = points[2] - points[1]
		local v3 = to - points[1]
		local n = v1:Cross(v2)
		return n:Dot(v3) < 0 and -n or n
	end
end

--The function that finds the intersection between two sets
--of points, s1 and s2. s1 and s2 must return the point in
--the set that is furthest in a given direction when called.
--If the start direction sV is specified as the seperation
--vector, the program runs in constant time. (excluding the
--user implemented functions for finding the furthest point).
function intersection(s1, s2, sV)
	local points = {}

	-- find point 
	local function support(dir)
		local a = s1(dir)
		local b = s2(-dir)
		points[#points + 1] = a - b
		return dir:Dot(a) < dir:Dot(b)
	end

	-- find all points forming a simplex
	if support(sV)
		or support(getDir(points, origin))
		or support(getDir(points, origin))
		or support(getDir(points, origin))
	then
		return false
	end

	local step, others, removed = 0, nil,nil
	repeat
		step, others, removed = loopRemoved(points, step)
		local dir = getDir(others, removed)
		if others[1]:Dot(dir) > 0 then
			points = others
			if support(-dir) then
				return false
			end
			step = 0
		end
	until step == 4

	return true
end

--Checks if two vectors are equal
local function equals(p1, p2)
	return p1.x == p2.x and p1.y == p2.y and p1.z == p2.z
end

--Gets the mathematical scalar t of the parametrc line defined by
--o + t * v of a point p on the line (the magnitude of the projection).
local function getT(o, v, p)
	return (p - o):Dot(v) / v:Dot(v)
end

--Returns the scalar of the closest point on a line to
--the origin. Note that if the vector is a zero vector then
--it treats it as a point offset instead of a line.
local function lineToOrigin(o, v)
	if equals(v, origin) then
		return o
	end
	local t = getT(o, v, origin)
	if t < 0 then 
		t = 0
	elseif t > 1 then 
		t = 1
	end
	return o + v*t
end

--Convoluted to deal with cases like points in the same place
local function closestPoint(a, b, c)
	--if abc is a line
	if c == nil then
		--get the scalar of the closest point
		local dir = b - a
		local t = getT(a, dir, origin)
		if t < 0 then t = 0
		elseif t > 1 then t = 1
		end
		--and return the point
		return a + dir * t
	end

	--Otherwise it is a triangle.
	--Define all the lines of the triangle and the normal
	local vAB, vBC, vCA = b - a, c - b, a - c
	local normal = vAB:Cross(vBC)

	--If two points are in the same place then
	if normal.magnitude == 0 then

		--Find the closest line between ab and bc to the origin (it cannot be ac)
		local ab = lineToOrigin(a, vAB)
		local bc = lineToOrigin(b, vBC)
		if ab.magnitude < bc.magnitude then
			return ab
		else
			return bc
		end

		--The following statements find the line which is closest to the origin
		--by using voroni regions. If it is inside the triangle, it returns the
		--normal of the triangle.
	elseif a:Dot(a + vAB * getT(a, vAB, c) - c) <= 0 then
		return lineToOrigin(a, vAB)
	elseif b:Dot(b + vBC * getT(b, vBC, a) - a) <= 0 then
		return lineToOrigin(b, vBC)
	elseif c:Dot(c + vCA * getT(c, vCA, b) - b) <= 0 then
		return lineToOrigin(c, vCA)
	else
		return -normal * getT(a, normal, origin)
	end
end

--The distance function. Works like the intersect function above. Returns
--the translation vector between the two closest points.
function distance(s1, s2, sV)
	local function support (dir)
		return s1(dir) - s2(-dir)
	end

	--Find the initial three points in the search direction, opposite of the
	--search direction, and in the orthoginal direction between those two 
	--points to the origin.
	local a = support(sV)
	local b = support(-a)
	local c = support(-closestPoint(a, b))

	--Setup maximum loops
	local i = 1
	while i < 100 do
		i = i + 1

		--Get the closest point on the triangle
		local p = closestPoint(a, b, c)

		--If it is the origin, the objects are just touching, 
		--return a zero vector.
		if equals(p, origin) then
			return origin
		end

		--Search in the direction from the closest point
		--to the origin for a point. 
		local dir = p.unit
		local d = support(dir)
		local dd = d:Dot(dir)
		local dm = math.min(
			a:Dot(dir),
			b:Dot(dir),
			c:Dot(dir)
		)

		--If the new point is farther or equal to the closest 
		--point on the triangle, then we have found the closest 
		--point.
		if dd >= dm then
			--return the point on the minkowski difference as the
			--translation vector between the two closest point.
			return -p
		end

		--Otherwise replace the point on the triangle furthest 
		--from the origin with the new point
		local ma, mb, mc = a:Dot(dir), b:Dot(dir), c:Dot(dir)
		if ma > mb then
			if ma > mc then
				a = d
			else
				c = d
			end
		elseif mb > mc then
			b = d
		else
			c = d
		end
	end

	--Return an error if no point was found in the maximum 
	--number of iterations
	error 'Unable to find distance, are they intersecting?'
end


local  VpfRayCast = {}

local Runservice = game:GetService("RunService")
local GJK = require(script.GJK)
local FRONT = Vector3.new(0, 0, -1)
local UP = Vector3.new(0,1,0)
local HitTestIncrement = .04
local DefaultRayLength = 1000
local min = math.min
local max = math.max
local AllParts = {}
 VpfRayCast.__index =  VpfRayCast


local function LocalPos(Pos,Viewport)
	local pos = Pos - Viewport.AbsolutePosition
	return pos
end


local function ToVec2(Vector)
	return Vector2.new(Vector.x, Vector.y)
end


local function ScreenTo3DPlane(Pos, viewport, camera,Depth, Normal, Gui_Inset)
 Pos =  LocalPos(Vector2.new(Pos.X, Pos.Y ), viewport)
 local Cam_Size = Vector2.new(viewport.AbsoluteSize.X , viewport.AbsoluteSize.Y - (Gui_Inset or 0))
 local Height = Cam_Size.Y
 local Width = Cam_Size.X	
 local AspectRatio = (Cam_Size.X/Cam_Size.Y)
 local Cam_Pos = camera.CFrame.Position 
 local Scale =(Depth or 1) 
 local yfov =math.rad(camera.FieldOfView)
local Tangent = math.tan((yfov/2));
 local fx = ((2 * Scale) * (Pos.x /(Width-1)) -(Scale*1))
 local fy = ((2 * Scale) * (Pos.y/(Height-1)) -(Scale*1))
 local NX = ((AspectRatio * Tangent * fx ))
 local NY = (-Tangent * fy)
local NZ = -Scale 
local Translatedcf = (camera.CFrame) * CFrame.new(Vector3.new(NX, NY, NZ))  -- rotate rel to camera
    return CFrame.new( Translatedcf.p , camera.CFrame.p  ) -- rotate to face camera
end 



local function GetWedgeCorners(part)
local T = {}
local S = part.size * .5
local cf = part.CFrame 
table.insert(T, cf * CFrame.new(-S.x,-S.y,-S.z))	
table.insert(T,cf * CFrame.new(S.x,-S.y,-S.z))	
table.insert(T, cf * CFrame.new(-S.x,S.y,S.z))	
table.insert(T,cf * CFrame.new(S.x,S.y,S.z))	
table.insert(T, cf * CFrame.new(-S.x,-S.y,S.z))	
table.insert(T,cf *CFrame.new(S.x,-S.y,S.z))	
   return T
end





local function PartSupport(part)
local s = part.Size * 0.5
  local cf = part.CFrame
  cf = cf - cf.p
  local pI = cf.RightVector * s.X
 local  pJ = cf.UpVector * s.Y
  local  pK = cf.LookVector * s.Z
  local  pPos = part.Position
     return function (dir)
	return pPos
	+ (pI:Dot(dir) > 0 and pI or -pI)
	+ (pJ:Dot(dir) > 0 and pJ or -pJ)
	+ (pK:Dot(dir) > 0 and pK or -pK)
  end
end



 local function SupportPolygon(vertices)
   return function(dir)
        local furthestDistance = -math.huge
        local furthestVertex = nil
        for i =1, #vertices do
            local distance =   vertices[i].p:Dot(dir);
            if (distance > furthestDistance) then
              furthestDistance = distance;
             furthestVertex = vertices[i].p
       end
    end
   return furthestVertex;
 end
end


local function SupportWedge(part)
	local vertices = GetWedgeCorners(part)
  return  SupportPolygon(vertices)
end



local function SupportPoint(pointvec)
 return function(dir)
 local cf = CFrame.new(pointvec) - pointvec
  local  SP_R, SP_U, SP_L = cf.RightVector , cf.UpVector , cf.LookVector 
	return pointvec
	+ (SP_R:Dot(dir) > 0 and SP_R or -SP_R)
	+ (SP_U:Dot(dir) > 0 and SP_U or -SP_U)
	+ (SP_L:Dot(dir) > 0 and SP_L or -SP_L)
   end
end


local function rejectionVector(point, start, dir)
	local vStartToPoint = point - start
	return vStartToPoint - vStartToPoint:Dot(dir) * dir
end

local function rejectionMagnitudeSquared(point, start, dir)
	local vStartToPoint = point - start
	local projectionMagnitude = vStartToPoint:Dot(dir)
	return vStartToPoint:Dot(vStartToPoint) - projectionMagnitude * projectionMagnitude
end



local function supportRay (rayStart, rayFinish)
 return function (dir)
	if rayStart:Dot(dir) > rayFinish:Dot(dir) then
		return rayStart
	else
		return rayFinish
	end
  end
end


local function SupportSphere(Part)
local pos = Part.Position
local s = Part.Size * 0.5
local radius = math.min(s.X, s.Y, s.Z)
  return function(dir)
	return pos + dir.Unit * radius
   end
end



		
local function Scale(vec, n)
   return Vector3.new(vec.X * n ,vec.Y * n, vec.Z * n )
end


local function OBBLineIntersection(OBB, Start, End )
 local Dir = End - Start
 local Delta = OBB.Position - Start
 local CF = OBB.CFrame
 local Bmax =  CF:PointToObjectSpace(CF * (OBB.Size/2)) 
 local Bmin = CF:PointToObjectSpace(CF * (-OBB.Size/2)) 
 local CurrAxis 
 local tMin = -math.huge
 local tMax = math.huge	
 local epsilon= 0.00001
  local p,nomLen,denomLen,tmp,min,max;
     for _, Axis in ipairs(Enum.Axis:GetEnumItems()) do
	  if Axis == Enum.Axis.X then
         CurrAxis  = CF.RightVector
		elseif Axis == Enum.Axis.Y then
	    CurrAxis  = CF.UpVector
		else
       CurrAxis = CF.LookVector
	end
	nomLen = Delta:Dot(CurrAxis)
      denomLen = CurrAxis:Dot(Dir)
	 if math.abs(denomLen) >  epsilon  then
		min = (nomLen + Bmin[Axis.Name] )/denomLen
		max = (nomLen + Bmax[Axis.Name]  )/denomLen
		if min > max then
			tmp = min; min = max; max = tmp
		end
	     if min > tMin then
          tMin = min
	   end
	        if max < tMax then
		    tMax = max
	        end
	       if tMax < tMin then
		return false
	     end
      	elseif  ((-nomLen + Bmin[Axis.Name] ) > 0) or (-nomLen +Bmax[Axis.Name]  <0) then
		return false
      end
  end
local FirstIntersect = Scale(Dir, tMin ) + Start
local LeavingIntersect = Scale(Dir, tMax ) + Start
    return true, FirstIntersect, LeavingIntersect
end




local function AABBLineIntersection(AABB, S, E)
local StoE = E - S
local P = AABB.Position
local Size = AABB.Size/2
local min = P - Size
local max = P + Size
local Bmin = min-S
local Bmax = max - S
local Near = -math.huge
local Far = math.huge
local intersect = false
local p1
local p2
  for _, Axis in ipairs(Enum.Axis:GetEnumItems()) do
  if StoE[Axis.Name] == 0 then
    if (Bmin[Axis.Name] > 0 ) or (Bmax[Axis.Name] < 0) then
	    return p1, p2
	  end
   else
  local t1 = Bmin[Axis.Name]/StoE[Axis.Name]
  local t2 = Bmax[Axis.Name]/StoE[Axis.Name]
  local tmin = math.min(t1, t2)
  local tmax = math.max(t1,t2)
  if tmin > Near then
   Near = tmin
end
 if tmax < Far then
	Far = tmax
end
if Near > Far or Far < 0 then
  return  intersect, p1, p2
	 end	
   end
end
  	if Near >= 0 and Near <= 1 then
     	p1 =  S + StoE * Near 
	end
	if Far >= 0 and Far <= 1 then
	p2 =  S + StoE * Far
   end
  if p1 and p2 then
	  intersect = true
end

    return  intersect, p1, p2
end

local function FindType(p , types)
	if p:IsA("Part") and (p.Shape == Enum.PartType.Ball) then
		return types["Ball"]
	end
	for i,v in pairs(types) do
		if p:IsA(i)  then
			return v
		end
	end
	return "Not Valid PartType"
end


------------------------------------------------------------------------------------------------------------------
local SupportFunctions = {BasePart = PartSupport,WedgePart = SupportWedge, Ball = SupportSphere } ---You can add more... (:




local function ClosetPart(RayOrigin, Rayend, dir , Parts, All)
local Best
local BestScore = math.huge
local Hit 
local intersections = {}
   for _, PartInfo in ipairs(Parts) do
    local part = PartInfo[1]
    local Pos = part.Position 
    local Size = part.Size
      local BBIntersect, Start, End 
	local Length = 0
	local partType = PartInfo[2]
	 local intersection = false
	if part.Orientation == Vector3.new() then 
      	 BBIntersect, Start, End  =  AABBLineIntersection(part,RayOrigin,  Rayend)
		else
	  BBIntersect, Start, End  = OBBLineIntersection(part,RayOrigin,  Rayend)
   end
	 if  BBIntersect then
	    while  not intersection do
 	       local pointvec = Start + dir * Length
          Length = Length +  HitTestIncrement
          if Length >= (Start - End).magnitude then
	         intersections[part] = Vector3.new(pointvec.X, pointvec.Y,pointvec.Z) 
             intersection = true
	      end
           if Intersection(partType(part), SupportPoint(pointvec), UP)   then
	           intersections[part] =  Vector3.new(pointvec.X, pointvec.Y,pointvec.Z) 
	          intersection = true
	   end
	 end
	end
  end

for _, v in  pairs(intersections) do
local mag = (RayOrigin - v ).Magnitude 
	if (mag  < BestScore) then
		BestScore = mag
	    Best  =_
	  Hit = v
	end
end

 if  All then
	return Parts, Hit
else
 	 return Best, Hit
   end
end



local function Raycast(start, dir,  parts,maxDist, ReturnAll )
maxDist = maxDist or DefaultRayLength or 500
local	finish = start + dir * maxDist 
   local Foundparts = {}
    for _, part in ipairs(parts) do
	local Mag = rejectionMagnitudeSquared(part.Position, start, dir)
     if Mag < part.Size:Dot(part.Size) then
             local SupportFunc = FindType(part, SupportFunctions)
			local  vec = rejectionVector(part.Position, start, dir)
	     	 if Intersection(SupportFunc(part) ,supportRay(start, finish),vec) then
			table.insert(Foundparts, {part, SupportFunc })
	      end
	    end
       end
        if ReturnAll then
       return Foundparts, finish
	else
	 return ClosetPart(start,  finish,dir,  Foundparts, ReturnAll)
   end
end







local function GetAllDecendents(Table, handler, Filter)
   if not Table[handler]  then
	Table[handler]  ={}
   end
  Filter = Filter or {}
local ModelTargetFilters = {}
	for _, part in ipairs(handler:GetDescendants()) do
	  if not table.find(Filter, part) and not table.find(ModelTargetFilters,part) then
		if part:IsA("MeshPart") or part:IsA("BasePart")  then
				if not table.find(Table[handler] , part) then
			   table.insert(Table[handler] , part)
			       end
			    end
			elseif part:IsA("Model") then
			 for _, p in pairs(part:GetDescendants()) do
				  table.insert(ModelTargetFilters, p)
			end
		end
	 end
  return Table[handler]
end


local function MouseHitFunc(Mouse,Viewport, Handler, cam, TargetFilter,All,Dist)
if typeof(TargetFilter) ~= "table" then TargetFilter = {TargetFilter} end
 local Parts = GetAllDecendents(AllParts, Handler or Viewport, TargetFilter) -----This Can Be Removed as to not check constantly for parts
    local cf, rot =  ScreenTo3DPlane(Vector2.new(Mouse.X, Mouse.Y) , Viewport, cam)
	local FowardDir = cf:VectorToWorldSpace(-FRONT)
	local Parts, Pos = Raycast(cf.p, FowardDir,Parts, Dist, All)
  return Parts, Pos
end



function VpfRayCast.NewMouse(Mouse,vpf,camera, Handler, maxDist )
  local Functions = {
MouseTarget = function(Targetfilter, All)
     return MouseHitFunc(Mouse,vpf, Handler, camera,Targetfilter, All, maxDist)
 end,
}
	return Functions, setmetatable(VpfRayCast,Functions)
end

VpfRayCast.new = VpfRayCast.NewMouse
VpfRayCast.RayCast = Raycast

return VpfRayCast
