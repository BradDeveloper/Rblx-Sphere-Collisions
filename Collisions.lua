--Helper Functions (Credit given where required)

--Credit:RobloxCoreGui
--Projects a Ray onto a Plane if intersection occurs.
function RayPlaneIntersection(ray: Ray, planeNorm: Vector3, pointOnPlane: Vector3): (Vector3 | nil)
	planeNorm = planeNorm.Unit
	ray = ray.Unit

	local Vd = planeNorm:Dot(ray.Direction)
	if Vd == 0 then -- parallel, no intersection
		return nil
	end

	local V0 = planeNorm:Dot(pointOnPlane - ray.Origin)
	local t = V0 / Vd
	if t < 0 then --plane is behind ray origin, and thus there is no intersection
		return nil
	end

	return ray.Origin + ray.Direction * t
end

--Credit: Marco13 https://math.stackexchange.com/a/1905794
--Returns the position and distance of point A projected onto line BvCv
function ProjectPointToLine(a: Vector3, bv: Vector3, cv: Vector3): (Vector3, number)
	local d = (cv - bv) / (cv - bv).Magnitude
	local v = a - bv
	local t = v:Dot(d)
	local p = bv + t*d
	return p, (p - a).Magnitude
end

function GetTriangleNormal(a: Vector3, b: Vector3, c: Vector3): Vector3
	return (b - a):Cross(c - b)
end

local function getSurfaceNormal(cf: CFrame, normalId: Vector3): Vector3
	return ((cf * Vector3.FromNormalId(normalId)) - cf.Position).Unit
end

--Collision Functions
--Handles a Box-Plane to Sphere Collision
--Returns new Vector3 outside of Box
function BoxSphereCollision(plane: BasePart, pos: Vector3, radius: number): Vector3
    local planeCFrame = plane.CFrame
    local planePos = planeCFrame.Position

    local normId = Enum.NormalId.Top
    local normVect = getSurfaceNormal(planeCFrame, normId)
    local surfRay = Ray.new(pos, normVect * -1)

    local planeCollision = RayPlaneIntersection(surfRay, normVect, planePos)
    if planeCollision then
        local planeSize = plane.Size * 0.5
        local surfaceOffsetVect = normVect * (planeSize.Y + radius)

        return planeCollision + surfaceOffsetVect
    end
    return pos
end

--Handles a Cylinder-Plane to Sphere Collision
--Returns new Vector3 outside of Box
function CylinderSphereCollision(plane: BasePart, pos: Vector3, radius: number): Vector3
    local planeCFrame = plane.CFrame
    local planePos = planeCFrame.Position
    
    local planeSize = plane.Size * 0.5
    local planeLength = planeSize.X
    local planeRadius = planeSize.Y

    local vertA = planePos + planeCFrame.RightVector * planeLength
    local vertB = planePos - planeCFrame.RightVector * planeLength
    local projectedPoint = ProjectPointToLine(pos, vertA, vertB)

    local normVect = pos - projectedPoint
    local normMagnitude = planeRadius  + radius

    --Roblox cylinder have box collision
    local isInSphere = normMagnitude > normVect.Magnitude
    if not isInSphere then
        return pos
    end

    return projectedPoint + (normVect.Unit * normMagnitude)
end

--Handles a Sphere to Sphere Collision
--Returns new Vector3 outside of Box
function SphereSphereCollision(plane: BasePart, pos: Vector3, radius: number): Vector3
    local planeCFrame = plane.CFrame
    local planePos = planeCFrame.Position

    local planeRadius = plane.Size.Y * 0.5

    local normVect = pos - planePos
    local normMagnitude = planeRadius + radius

    --Roblox spheres have box collision
    local isInSphere = normMagnitude > normVect.Magnitude
    if not isInSphere then
        return pos
    end
    return planePos + (normVect.Unit * normMagnitude)
end

--Handles a Wedge-Plane to Sphere Collision
--Returns new Vector3 outside of Box
function WedgeSphereCollision(plane: BasePart, pos: Vector3, radius: number): Vector3
    local planeCFrame = plane.CFrame
    local planePos = planeCFrame.Position
    local planeSize = plane.Size * 0.5

    local vectLook = planeCFrame.LookVector
    local vectUp = planeCFrame.UpVector
    local vectRight = planeCFrame.RightVector
    local vertB = planePos + (vectUp * planeSize.Y) - (vectLook * planeSize.Z)
    local vertA = planePos - (vectUp * planeSize.Y) + (vectLook * planeSize.Z)
    local planeNorm = (vertB - vertA).Unit:Cross(vectRight)

    local surfRay = Ray.new(pos, planeNorm * -1)
    local planeCollision = RayPlaneIntersection(surfRay, planeNorm, planePos)

    if planeCollision then
        --Roblox wedges have box collision
        local isInWedge = (planeCollision - pos).Magnitude < radius
        if isInWedge then
            local surfaceOffsetVect = planeNorm * radius
            return planeCollision + surfaceOffsetVect
        end
    end
    return pos
end

--Handles a CornerWedge-Plane to Sphere Collision
--Returns new Vector3 outside of Box
function CornerWedgeSphereCollision(plane: BasePart, pos: Vector3, radius: number): Vector3
    local planeCFrame = plane.CFrame
    local planePos = planeCFrame.Position
    local planeSize = plane.Size * 0.5

    local vectLook = planeCFrame.LookVector
    local vectUp = planeCFrame.UpVector
    local vectRight = planeCFrame.RightVector
    local vertA = planePos + (vectRight * planeSize.X) + (vectUp * planeSize.Y) + (vectLook * planeSize.Z)
    local vertB = planePos - (vectRight * planeSize.X) - (vectUp * planeSize.Y) + (vectLook * planeSize.Z)
    local vertC = planePos - (vectRight * planeSize.X) - (vectUp * planeSize.Y) - (vectLook * planeSize.Z)
    local vertD = planePos + (vectRight * planeSize.X) - (vectUp * planeSize.Y) - (vectLook * planeSize.Z)
    
    local vectCA = (vertC - vertA).Unit

    local function getPlaneCollide(planeNorm)
        local surfRay = Ray.new(pos, planeNorm * -1)
        local planeCollision = RayPlaneIntersection(surfRay, planeNorm, planePos)

        if planeCollision then
            --Roblox wedges have box collision
            local isInWedge = (planeCollision - pos).Magnitude < radius
            if isInWedge then
                local surfaceOffsetVect = planeNorm * radius
                return planeCollision + surfaceOffsetVect
            end
        end
        return pos
    end

    --Find which face to check collisions for
    local cframeCA = CFrame.fromMatrix(planePos, vectCA, planeCFrame.UpVector)
    local isLeftFace = cframeCA:PointToObjectSpace(pos).Z > 0
    if isLeftFace then
        local normABC = GetTriangleNormal(vertA, vertB, vertC).Unit
        return getPlaneCollide(normABC)
    else
        local normACD = GetTriangleNormal(vertA, vertC, vertD).Unit
        return getPlaneCollide(normACD)
    end
end

return {
    BoxSphereCollision = BoxSphereCollision,
    SphereSphereCollision = SphereSphereCollision,
    CylinderSphereCollision = CylinderSphereCollision,
    WedgeSphereCollision = WedgeSphereCollision,
    CornerWedgeSphereCollision = CornerWedgeSphereCollision,
}
