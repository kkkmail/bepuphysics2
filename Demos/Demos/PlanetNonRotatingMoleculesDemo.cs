using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Reflection;
using System.Text;

namespace Demos.Demos;

/// <summary>
/// Shows how to use custom velocity integration to implement planetary gravity.
/// </summary>
public class PlanetNonRotatingMoleculesDemo : Demo
{
    #region Control Parameters

    private bool moveWall = false;

    #endregion

    #region Camera and Output

    private string prefix =>
        $"{(moveWall ? "Piston" : "Static")}" +
        "_R" + $"{(int)(moleculeRadius * 1000)}".PadLeft(4, '0') +
        "_V" + $"{(int)mainMoleculeVelocity}".PadLeft(3, '0') +
        (moveWall ? "_W" + $"{(int)wallSpeed}".PadLeft(3, '0') : "");

    private const string exportFolder = "C:\\PlanetNonRotatingMoleculesDemo";
    // private Vector3 cameraPosition = new Vector3(-110, 80, -50);
    // private Vector3 cameraPosition = new Vector3(-110, 80, -50) * 7;
    private Vector3 cameraPosition = new Vector3(-60, 90, -30) * 25;

    #endregion

    #region Box with molecules

    const float moleculeRadius = 2.000f;
    const float mainMoleculeVelocity = 20f;

    // const int count = 40;
    const int count = 50;

    private const float minSpacingDistance = 5f;

    #region mainMoleculeVelocity = 20f;

    // float mainMoleculeVelocity = 20f;
    // const float moleculeRadius = 1.000f;

    int velocityIterationCount = 8;
    int substepCount = 1;
    float frequency = 5.0f;
    // float dampingRatio = -0.2625f;
    // float dampingRatio = -0.2623f;
    // float dampingRatio = -0.2622f;
    // float dampingRatio = -0.2621f;

    // float dampingRatio = -0.2620f; // 10 minutes: 50 -> 51.31
    // float dampingRatio = -0.2619f; // 10 minutes: 50 -> 50.75
    float dampingRatio = -0.2618f; // 10 minutes: 50 -> 50.28

    // float dampingRatio = -0.26179f; // 10 minutes: 50 -> goes below 50
    // float dampingRatio = -0.26175f; // 10 minutes: 50 -> goes below 50
    // float dampingRatio = -0.2617f; // 10 minutes: 50 -> goes below 50

    #endregion

    #region mainMoleculeVelocity = 50f;

    // const float mainMoleculeVelocity = 50f;
    // const float moleculeRadius = 0.25f;
    //
    // int velocityIterationCount = 8;
    // int substepCount = 1;
    // float frequency = 5.0f;
    // float dampingRatio = -0.2618f;

    #endregion

    #endregion

    #region Physical Parameters

    private float gravityValue;

    // float frequency = 30.0f;
    // float dampingRatio = 0.0f;

    float maximumRecoveryVelocity = float.MaxValue;
    float frictionCoefficient = 0.0f;

    #region Solid Planet - velocityIterationCount: 8, substepCount: 1, frequency: 5.0

    // int velocityIterationCount = 8;
    // int substepCount = 1;
    // float frequency = 5.0f;
    // float dampingRatio = -0.2625f; // Has runaway orbiters.
    // // float dampingRatio = -0.262f;

    #endregion

    #region Mesh Planet - velocityIterationCount: 8, substepCount: 1, frequency: 5.0

    float thickness = 5.0f;
    int numberOfLayers = 10;

    // int velocityIterationCount = 8;
    // int substepCount = 1;
    // float frequency = 5.0f;
    // float dampingRatio = 0f;

    #endregion

    // int velocityIterationCount = 8;
    // int substepCount = 8;

    // int velocityIterationCount = 16;
    // int substepCount = 16;
    // float frequency = 30.0f;
    // float dampingRatio = -0.09817f; // 199.99846 of 200
    // float dampingRatio = -0.0981703f; // 199.9985937 of 200

    // int velocityIterationCount = 32;
    // int substepCount = 32;
    // float frequency = 30.0f;
    // // float dampingRatio = -0.05f; // -19.8445 of 20; -193.224 of 200.
    // float dampingRatio = -0.049f; // Passes through at 200 with +199.986 and even passes through at 20.

    // int velocityIterationCount = 64;
    // int substepCount = 64;
    // float frequency = 300.0f;
    // float dampingRatio = -0.245f; // -19.36 at +20

    float activity = 0.01f;

    #endregion

    #region Planetary Parameters

    static float testVelocityValue = 0f;

    static float testOriginValue = 500f;
    Vector3 testOrigin = new Vector3(-testOriginValue, 0, 0);
    Vector3 testOrigin2 = new Vector3(testOriginValue, 0, 0);
    Vector3 testVelocity = new Vector3(testVelocityValue, 0, 0);
    Vector3 testVelocity2 = new Vector3(-testVelocityValue, 0, 0);

    float PlanetRadius = 50.0f;
    Vector3 PlanetCenter = new Vector3();
    int subDivisionSteps = 8;

    float orbiterRadius = moleculeRadius;
    float orbiterMass = moleculeMass;

    private static int seed = 1;
    const float moleculeMass = moleculeRadius * moleculeRadius * moleculeRadius;

    Random random = new Random(seed);

    private static float spacingDistance = Math.Max(3 * moleculeRadius, minSpacingDistance);
    private Vector3 spacing = new Vector3(spacingDistance);

    const int length = count;
    const int width = count;
    const int height = count;

    private const int planetMeshWidth = 20;

    Vector3 mainOrigin = new Vector3(-200, 300, 0);
    Vector3 mainMoleculeOrigin = new Vector3();

    Vector3 mainVelocity = new Vector3();

    StaticHandle PlanetHandle;

    Vector3 position = new Vector3(0, 0, 0);
    Quaternion rotation = QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2);
    Vector3 scalingVector = new Vector3(1, 1, 1);

    private BodyHandle testHandle;
    private bool hasTestOrbiter;

    private BodyHandle[] orbiterHandles;
    private bool hasOrbiters;

    private BodyHandle[] moleculeHandles;
    private bool hasMolecules;

    # region Statistics

    private int orbiterStatisticsCallCount = -1;
    private int orbiterStatisticsReportingFrequency = 50;
    private Vector3 averageSpeed = Vector3.Zero;
    private float averageAbsoluteSpeed;
    private Vector3 averageAngularSpeed = Vector3.Zero;
    private float averageAbsoluteAngularSpeed;
    private Vector3 averagePosition = Vector3.Zero;
    private float averageAbsolutePosition;
    private float averageKineticEnergy;
    private float averagePotentialEnergy;
    private float averageTotalEnergy;
    private float minimumAbsolutePosition;
    private float maximumAbsolutePosition;
    private int orbitersInsidePlanet;
    private int runawayOrbiters;
    private Vector3 averageAngularMomentum = Vector3.Zero;
    private float averageAbsoluteAngularMomentum;

    #endregion

    #endregion

    #region Box Parameters

    private bool hasBox;

    // private const float boxWallThickness = 2f;
    // private const float boxWidth = 400f;
    // private const float boxLength = 400f;
    // private const float boxHeight = 400f;

    private const float boxWallThickness = 1_000f;
    private const float boxWidthInternal = 600f;
    private const float boxLengthInternal = 600f;
    private const float boxHeightInternal = 600f;

    // private const float boxWallThickness = 2f;
    // private const float boxWidthInternal = 100f;
    // private const float boxLengthInternal = 100f;
    // private const float boxHeightInternal = 100f;

    float boxWidth = boxWidthInternal + boxWallThickness;
    float boxHeight = boxHeightInternal + boxWallThickness;
    float boxLength = boxLengthInternal + boxWallThickness;

    private BodyHandle topWallHandle;

    // private const float wallMovementTime = 40f;
    // private const float wallStaticTime = 20f;

    // private const float wallMovementTime = 30 * 60f;
    // private const float wallStaticTime = 1 * 60f;

    private const float wallMovementTime = 30f;
    private const float wallStaticTime = 90f;
    private const float wallFirstStaticTime = 300f;

    private const float wallPeriodTime = 2 * (wallMovementTime + wallStaticTime);
    private const float wallMovementLength = 2 * boxHeightInternal / 4;
    private const float wallSpeed = wallMovementLength / wallMovementTime;
    private float wallCurrentSpeed;
    private float wallCurrentPosition;

    #endregion

    #region Callback

    struct PlanetaryGravityCallbacks : IPoseIntegratorCallbacks
    {
        public Vector3 PlanetCenter;
        public float Gravity;
        public float Radius;

        public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

        public readonly bool AllowSubstepsForUnconstrainedBodies => false;

        public readonly bool IntegrateVelocityForKinematics => false;

        public void Initialize(Simulation simulation)
        {
        }

        float gravityDt;

        public void PrepareForIntegration(float dt)
        {
            //No point in repeating this for every body; cache it.
            gravityDt = dt * Gravity;
        }

        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation,
            BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt,
            ref BodyVelocityWide velocity)
        {
            var offset = position - Vector3Wide.Broadcast(PlanetCenter);
            var distance = offset.Length();

            if (distance[0] >= Radius)
            {
                velocity.Linear -= new Vector<float>(gravityDt) * offset /
                                   Vector.Max(Vector<float>.One, distance * distance * distance);
            }
        }
    }

    #endregion

    #region SimulationStatistics

    private readonly List<SimulationStatistics> statisticsHistory = new();

    public record SimulationStatistics
    {
        public required double RealTime { get; init; }
        public required double SimulationTime { get; init; }
        public required float WallCurrentSpeed { get; init; }
        public required float WallCurrentPosition { get; init; }
        public required int OrbiterStatisticsCallCount { get; init; }
        public required float AverageSpeedX { get; init; }
        public required float AverageSpeedY { get; init; }
        public required float AverageSpeedZ { get; init; }
        public required float AverageAbsoluteSpeed { get; init; }
        public required float AverageAngularSpeedX { get; init; }
        public required float AverageAngularSpeedY { get; init; }
        public required float AverageAngularSpeedZ { get; init; }
        public required float AverageAbsoluteAngularSpeed { get; init; }
        public required float AveragePositionX { get; init; }
        public required float AveragePositionY { get; init; }
        public required float AveragePositionZ { get; init; }
        public required float AverageAbsolutePosition { get; init; }
        public required float AverageKineticEnergy { get; init; }
        public required float AveragePotentialEnergy { get; init; }
        public required float AverageTotalEnergy { get; init; }
        public required float MinimumAbsolutePosition { get; init; }
        public required float MaximumAbsolutePosition { get; init; }
        public required int OrbitersInsidePlanet { get; init; }
        public required int RunawayOrbiters { get; init; }
        public required float AverageAngularMomentumX { get; init; }
        public required float AverageAngularMomentumY { get; init; }
        public required float AverageAngularMomentumZ { get; init; }
        public required float AverageAbsoluteAngularMomentum { get; init; }
    }

    #endregion

    #region Helper Methods

    private void SetCamera(Camera camera)
    {
        camera.Position = cameraPosition;
        camera.Yaw = 0;
        camera.Pitch = MathF.PI * -0.5f;
    }

    private void SetGravity()
    {
        gravityValue = 100_000.0f;
    }

    private void CreateSimulation(Camera camera)
    {
        SetCamera(camera);

        Simulation = Simulation.Create(BufferPool,
            new DemoNarrowPhaseCallbacks(
                new SpringSettings(frequency, dampingRatio),
                maximumRecoveryVelocity: maximumRecoveryVelocity,
                frictionCoefficient: frictionCoefficient),
            new PlanetaryGravityCallbacks
                { PlanetCenter = PlanetCenter, Gravity = gravityValue, Radius = PlanetRadius },
            new SolveDescription(velocityIterationCount: velocityIterationCount, substepCount: substepCount));
    }

    private void CreatePlanet()
    {
        PlanetHandle =
            Simulation.Statics.Add(new StaticDescription(PlanetCenter,
                Simulation.Shapes.Add(new Sphere(PlanetRadius))));
    }

    private void CreateBox()
    {
        hasBox = true;
        boxWidth = boxWidthInternal + boxWallThickness;
        boxHeight = boxHeightInternal + boxWallThickness;
        boxLength = boxLengthInternal + boxWallThickness;
        wallCurrentPosition = boxHeight / 2;

        #region Left & Right

        // Left
        var wall1Shape = new Box(width: boxWidth + boxWallThickness, height: boxHeight + boxWallThickness, length: boxWallThickness);
        var wall1Position = new Vector3(0, 0, -boxLength / 2);
        Simulation.Statics.Add(new StaticDescription(wall1Position, Simulation.Shapes.Add(wall1Shape)));

        // Right
        var wall2Shape = new Box(width: boxWidth + boxWallThickness, height: boxHeight + boxWallThickness, length: boxWallThickness);
        var wall2Position = new Vector3(0, 0, boxLength / 2);
        Simulation.Statics.Add(new StaticDescription(wall2Position, Simulation.Shapes.Add(wall2Shape)));

        #endregion

        #region Back & Front

        // Back
        var wall3Shape = new Box(width: boxWallThickness, height: boxHeight + boxWallThickness, length: boxLength + boxWallThickness);
        var wall3Position = new Vector3(boxWidth / 2, 0, 0);
        Simulation.Statics.Add(new StaticDescription(wall3Position, Simulation.Shapes.Add(wall3Shape)));

        // Front
        var wall4Shape = new Box(width: boxWallThickness, height: boxHeight + boxWallThickness, length: boxLength + boxWallThickness);
        var wall4Position = new Vector3(-boxWidth / 2, 0, 0);
        Simulation.Statics.Add(new StaticDescription(wall4Position, Simulation.Shapes.Add(wall4Shape)));

        #endregion

        #region Bottom & Top

        // Bottom
        var wall5Shape = new Box(width: boxWidth + boxWallThickness, height: boxWallThickness, length: boxLength + boxWallThickness);
        var wall5Position = new Vector3(0, -boxHeight / 2, 0);
        Simulation.Statics.Add(new StaticDescription(wall5Position, Simulation.Shapes.Add(wall5Shape)));

        #region Top

        // var wall6Shape = new Box(width: boxWidth + boxWallThickness, height: boxWallThickness, length: boxLength + boxWallThickness);
        // var wall6Position = new Vector3(0, boxHeight / 2, 0);
        // Simulation.Statics.Add(new StaticDescription(wall6Position, Simulation.Shapes.Add(wall6Shape)));

        // var wall6Shape = new Box(width: boxWidth - 1.1f * boxWallThickness, height: boxWallThickness, length: boxLength - 1.1f * boxWallThickness);
        var wall6Shape = new Box(width: boxWidth + boxWallThickness, height: boxWallThickness, length: boxLength + boxWallThickness);
        var wall6Index = Simulation.Shapes.Add(wall6Shape);
        var wall6Position = new Vector3(0, boxHeight / 2, 0);
        topWallHandle = Simulation.Bodies.Add(BodyDescription.CreateKinematic((wall6Position, default), wall6Index, -1));

        #endregion

        #endregion
    }

    private void MoveTopWall(float inverseDt)
    {
        if (!hasBox || !moveWall || realTime < wallFirstStaticTime)
        {
            wallCurrentSpeed = 0;
            return;
        }

        var body = Simulation.Bodies[topWallHandle];

        var wallTime = (realTime - wallFirstStaticTime + wallStaticTime) % wallPeriodTime;

        if (wallTime is < wallStaticTime or >= wallStaticTime + wallMovementTime and < 2 * wallStaticTime + wallMovementTime)
        {
            body.Velocity.Linear = default;
            wallCurrentSpeed = 0;
            return;
        }

        Vector3 targetPosition;
        if (wallTime is >= wallStaticTime and <= wallStaticTime + wallMovementTime)
        {
            wallCurrentSpeed = wallSpeed;
            wallCurrentPosition = (boxHeight / 2) - (float)(wallMovementLength * (wallTime - wallStaticTime) / wallMovementTime);
            targetPosition = new Vector3(0, wallCurrentPosition, 0);
        }
        else
        {
            wallCurrentSpeed = -wallSpeed;
            wallCurrentPosition = (boxHeight / 2) - wallMovementLength + (float)(wallMovementLength *
                (wallTime - (2 * wallStaticTime + wallMovementTime)) / wallMovementTime);

            targetPosition = new Vector3(0, wallCurrentPosition, 0);
        }

        //Since it's a kinematic body, we'll compute the current pose error, and then the velocity to correct that error within a single frame.
        body.Velocity.Linear = (targetPosition - body.Pose.Position) * inverseDt;

    }

    private void CreateMeshCylinder()
    {
        const float scale = 2.5f;

        // var position = new Vector3(0, -15, 0);
        var position = new Vector3(0, 0, 0);
        var rotation = QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2);
        var scalingVector = new Vector3(1, 1, 1);

        var middle = (planetMeshWidth - 1.0) / 2.0;
        var terrainPosition = new Vector2(1 - planetMeshWidth, 1 - planetMeshWidth) * scale * 0.5f;

        // void createMesh(bool positive)
        // {
        //     var planeMesh = DemoMeshHelper.CreateDeformedPlane(planetMeshWidth, planetMeshWidth,
        //         (int vX, int vY) =>
        //         {
        //             var terrainHeight = (positive ? 1 : -1) * (float)Math.Sqrt(middle * middle - vX * vX) * scale;
        //             var vertexPosition = new Vector2(vX * scale, vY * scale) + terrainPosition;
        //             return new Vector3(vertexPosition.X, terrainHeight, vertexPosition.Y);
        //         }, scalingVector, BufferPool, ThreadDispatcher);
        //     Simulation.Statics.Add(new StaticDescription(position, rotation, Simulation.Shapes.Add(planeMesh)));
        // }
        //
        // createMesh(true);
        // createMesh(false);

        var planeMesh1 = DemoMeshHelper.CreateDeformedPlane(planetMeshWidth, planetMeshWidth,
            (int vX, int vY) =>
            {
                var terrainHeight = (float)Math.Sqrt(middle * middle - vX * vX) * scale;
                var vertexPosition = new Vector2(vX * scale, vY * scale) + terrainPosition;
                return new Vector3(vertexPosition.X, terrainHeight, vertexPosition.Y);
            }, scalingVector, BufferPool, ThreadDispatcher);
        Simulation.Statics.Add(new StaticDescription(position, rotation, Simulation.Shapes.Add(planeMesh1)));

        var planeMesh2 = DemoMeshHelper.CreateDeformedPlane(planetMeshWidth, planetMeshWidth,
            (int vX, int vY) =>
            {
                var terrainHeight = -(float)Math.Sqrt(middle * middle - vX * vX) * scale;
                var vertexPosition = new Vector2(vX * scale, vY * scale) + terrainPosition;
                return new Vector3(vertexPosition.X, terrainHeight, vertexPosition.Y);
            }, scalingVector, BufferPool, ThreadDispatcher);
        Simulation.Statics.Add(new StaticDescription(position, rotation, Simulation.Shapes.Add(planeMesh2)));
    }

    private void CreateMeshSphere()
    {
        var planetMesh = PlanetMeshCreator.CreatePlanetMesh(PlanetRadius, subDivisionSteps, scalingVector, BufferPool,
            ThreadDispatcher);
        Simulation.Statics.Add(new StaticDescription(position, rotation, Simulation.Shapes.Add(planetMesh)));
    }

    private void CreateThickMeshSphere()
    {
        var planetMeshes = PlanetMeshCreator.CreateThickPlanetMesh(
            thickness: thickness,
            numberOfLayers: numberOfLayers,
            radius: PlanetRadius,
            subdivisionSteps: subDivisionSteps,
            scaling: scalingVector,
            BufferPool,
            ThreadDispatcher);

        foreach (var planetMesh in planetMeshes)
        {
            var shapeIndex = Simulation.Shapes.Add(planetMesh);
            Simulation.Statics.Add(new StaticDescription(position, rotation, shapeIndex));
        }
    }

    private void CreateThickPlate()
    {
        var meshes = RectangleMeshCreator.CreateThickRectangleMesh(
            thickness: thickness,
            numberOfLayers: numberOfLayers,
            skewedness: 2,
            width: PlanetRadius / 4,
            height: PlanetRadius / 4,
            scaling: scalingVector,
            BufferPool,
            ThreadDispatcher);

        foreach (var mesh in meshes)
        {
            var shapeIndex = Simulation.Shapes.Add(mesh);
            Simulation.Statics.Add(new StaticDescription(position, rotation, shapeIndex));
        }
    }

    private void CreateTestOrbiter()
    {
        hasTestOrbiter = true;
        var orbiter = new Sphere(orbiterRadius);
        var inertia = orbiter.ComputeInertia(orbiterMass);
        var orbiterShapeIndex = Simulation.Shapes.Add(orbiter);
        testHandle =
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(testOrigin, testVelocity, inertia, orbiterShapeIndex,
                activity));
    }

    private BodyHandle CreateTestOrbiter2()
    {
        var orbiter = new Sphere(orbiterRadius);
        var inertia = orbiter.ComputeInertia(orbiterMass);
        var orbiterShapeIndex = Simulation.Shapes.Add(orbiter);

        var handle =
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(testOrigin2, testVelocity2, inertia, orbiterShapeIndex,
                activity));
        return handle;
    }

    private void CreateOrbiters()
    {
        hasOrbiters = true;
        var orbiter = new Sphere(orbiterRadius);
        var inertia = orbiter.ComputeInertia(orbiterMass);
        var orbiterShapeIndex = Simulation.Shapes.Add(orbiter);

        orbiterHandles = new BodyHandle[length * height * width];

        for (var i = 0; i < length; ++i)
        {
            for (var j = 0; j < height; ++j)
            {
                var origin = mainOrigin + spacing * new Vector3(length * -0.5f, 0, width * -0.5f);
                for (var k = 0; k < width; ++k)
                {
                    orbiterHandles[k * length * height + j * length + i] = Simulation.Bodies.Add(
                        BodyDescription.CreateDynamic(
                            origin + new Vector3(i, j, k) * spacing, mainVelocity, inertia, orbiterShapeIndex,
                            activity));
                }
            }
        }
    }

    private void CreateMolecules()
    {
        hasMolecules = true;
        var molecule = new Sphere(moleculeRadius);
        var moleculeInertia = molecule.ComputeInertia(moleculeMass);
        var moleculeShapeIndex = Simulation.Shapes.Add(molecule);
        moleculeHandles = new BodyHandle[length * height * width];

        for (var i = 0; i < length; ++i)
        {
            for (var j = 0; j < height; ++j)
            {
                var origin = mainMoleculeOrigin + spacing * new Vector3(length * -0.5f, 0, width * -0.5f);

                for (var k = 0; k < width; ++k)
                {
                    var moleculeVelocity =
                        mainMoleculeVelocity * new Vector3((float)(random.NextDouble() - 0.5), (float)(random.NextDouble() - 0.5), (float)(random.NextDouble() - 0.5));

                    moleculeHandles[k * length * height + j * length + i] = Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                        origin + new Vector3(i, j, k) * spacing, moleculeVelocity, moleculeInertia,
                        moleculeShapeIndex, activity));
                }
            }
        }
    }

    private string GetParametersString()
    {
        var sb = new StringBuilder();
        var type = this.GetType();
        var fields = type.GetFields(BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static);

        foreach (var field in fields)
        {
            if ((field.FieldType == typeof(int) || field.FieldType == typeof(float)) &&
                field.IsLiteral == false) // Skip const fields
            {
                var value = field.GetValue(field.IsStatic ? null : this);
                if (value != null) // Only include fields that have been assigned values
                {
                    sb.AppendLine($"{field.Name}, {value}");
                }
            }
        }

        // Add const fields manually since they have assigned values
        sb.AppendLine($"count, {count}");
        sb.AppendLine($"minSpacingDistance, {minSpacingDistance}");
        sb.AppendLine($"moleculeRadius, {moleculeRadius}");
        sb.AppendLine($"testVelocityValue, {testVelocityValue}");
        sb.AppendLine($"testOriginValue, {testOriginValue}");
        sb.AppendLine($"seed, {seed}");
        sb.AppendLine($"moleculeMass, {moleculeMass}");
        sb.AppendLine($"length, {length}");
        sb.AppendLine($"width, {width}");
        sb.AppendLine($"height, {height}");
        sb.AppendLine($"planetMeshWidth, {planetMeshWidth}");
        sb.AppendLine($"boxWallThickness, {boxWallThickness}");
        sb.AppendLine($"boxWidthInternal, {boxWidthInternal}");
        sb.AppendLine($"boxLengthInternal, {boxLengthInternal}");
        sb.AppendLine($"boxHeightInternal, {boxHeightInternal}");
        sb.AppendLine($"wallMovementTime, {wallMovementTime}");
        sb.AppendLine($"wallStaticTime, {wallStaticTime}");
        sb.AppendLine($"wallFirstStaticTime, {wallFirstStaticTime}");
        sb.AppendLine($"wallPeriodTime, {wallPeriodTime}");
        sb.AppendLine($"wallSpeed, {wallSpeed}");
        sb.AppendLine($"wallMovementLength, {wallMovementLength}");

        return sb.ToString();
    }

    private void CollectStatistics()
    {
        var stats = new SimulationStatistics
        {
            RealTime = realTime,
            SimulationTime = simulationTime,
            WallCurrentSpeed = wallCurrentSpeed,
            WallCurrentPosition = wallCurrentPosition,
            OrbiterStatisticsCallCount = orbiterStatisticsCallCount,
            AverageSpeedX = averageSpeed.X,
            AverageSpeedY = averageSpeed.Y,
            AverageSpeedZ = averageSpeed.Z,
            AverageAbsoluteSpeed = averageAbsoluteSpeed,
            AverageAngularSpeedX = averageAngularSpeed.X,
            AverageAngularSpeedY = averageAngularSpeed.Y,
            AverageAngularSpeedZ = averageAngularSpeed.Z,
            AverageAbsoluteAngularSpeed = averageAbsoluteAngularSpeed,
            AveragePositionX = averagePosition.X,
            AveragePositionY = averagePosition.Y,
            AveragePositionZ = averagePosition.Z,
            AverageAbsolutePosition = averageAbsolutePosition,
            AverageKineticEnergy = averageKineticEnergy,
            AveragePotentialEnergy = averagePotentialEnergy,
            AverageTotalEnergy = averageTotalEnergy,
            MinimumAbsolutePosition = minimumAbsolutePosition,
            MaximumAbsolutePosition = maximumAbsolutePosition,
            OrbitersInsidePlanet = orbitersInsidePlanet,
            RunawayOrbiters = runawayOrbiters,
            AverageAngularMomentumX = averageAngularMomentum.X,
            AverageAngularMomentumY = averageAngularMomentum.Y,
            AverageAngularMomentumZ = averageAngularMomentum.Z,
            AverageAbsoluteAngularMomentum = averageAbsoluteAngularMomentum
        };

        statisticsHistory.Add(stats);
    }

    private void ExportToCsv(string prefix, string outputFolder)
    {
        Directory.CreateDirectory(outputFolder);

        var timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        var fileName = $"{prefix}__{timestamp}.csv";
        var filePath = Path.Combine(outputFolder, fileName);

        using var writer = new StreamWriter(filePath);

        // Write parameters
        writer.Write(GetParametersString());
        writer.WriteLine();

        // Write header
        var properties = typeof(SimulationStatistics).GetProperties();
        var headerParts = new string[properties.Length];
        for (int i = 0; i < properties.Length; i++)
        {
            headerParts[i] = properties[i].Name;
        }
        writer.WriteLine(string.Join(",", headerParts));

        // Write statistics data
        foreach (var stats in statisticsHistory)
        {
            var values = new string[properties.Length];
            for (int i = 0; i < properties.Length; i++)
            {
                var value = properties[i].GetValue(stats);
                values[i] = value?.ToString() ?? "";
            }
            writer.WriteLine(string.Join(",", values));
        }
    }

    #endregion

    #region Initialize

    public override void Initialize(ContentArchive content, Camera camera)
    {
        // SetGravity();
        CreateSimulation(camera);

        #region Planet

        // CreatePlanet();
        // CreateMeshCylinder();
        // CreateMeshSphere();

        // CreateThickMeshSphere();
        // CreateThickPlate();

        #endregion

        #region Box

        CreateBox();

        #endregion

        #region Orbiters

        // CreateTestOrbiter();
        // CreateTestOrbiter2();

        // CreateOrbiters();
        CreateMolecules();

        #endregion
    }

    #endregion

    #region Update

    double realTime;
    double simulationTime;

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        Simulation.Timestep(TimestepDuration, ThreadDispatcher);
        MoveTopWall(1f / TimestepDuration);
        simulationTime += TimestepDuration;
        realTime += dt;
    }

    #endregion

    #region Render

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        OutputTestVelocity(renderer, text, font);
        OutputOrbiterStatistics(renderer, text, font);

        var bottomY = renderer.Surface.Resolution.Y;
        renderer.TextBatcher.Write(
            text.Clear().Append("The library does not prescribe any particular kind of gravity."),
            new Vector2(16, bottomY - 48), 16, Vector3.One, font);
        renderer.TextBatcher.Write(
            text.Clear()
                .Append(
                    "The IPoseIntegratorCallbacks provided to the simulation is responsible for telling the simulation how to integrate."),
            new Vector2(16, bottomY - 32), 16, Vector3.One, font);
        renderer.TextBatcher.Write(
            text.Clear().Append($"Real time: {realTime:F2}, simulation time: {simulationTime:F2}, wall current speed: {wallCurrentSpeed:F4}, wall top speed: {wallSpeed:F4}."),
            new Vector2(16, bottomY - 16), 16, Vector3.One, font);

        // var deviceContext = renderer.Surface.Context;
        // var surface = renderer.Surface;
        // var planetStatic = Simulation.Statics.GetDescription(PlanetHandle);
        // DemoRenderer.Helpers.PackOrientation(planetStatic.Pose.Orientation, out var packedOrientation);
        // var color = new Vector3(100, 100, 100);
        //
        // // var shapeIndex = planetStatic.Shape.Index;
        // // var shape = Simulation.Shapes[shapeIndex];
        //
        // // var sphere = (Sphere)shape;
        //
        // // if (shape is Sphere sphere)
        // // {
        // // var instance = new SphereInstance(planetStatic.Pose, new Vector3(0.8f, 0.8f, 0.8f)); // Gray color
        //
        // var instance = new SphereInstance
        // {
        //     Position = PlanetCenter,
        //     Radius = PlanetRadius + 100,
        //     PackedOrientation = packedOrientation,
        //     PackedColor = DemoRenderer.Helpers.PackColor(color),
        // };
        //
        // renderer.SphereRenderer.Render(deviceContext, camera, surface.Resolution, [instance], 0, 1);
        base.Render(renderer, camera, input, text, font);
    }

    private void OutputTestVelocity(Renderer renderer, TextBuilder text, Font font)
    {
        if (!hasTestOrbiter)
        {
            return;
        }

        Simulation.Bodies.GetDescription(testHandle, out var description);
        var velocity = description.Velocity.Linear;
        var absoluteVelocity = Math.Sqrt(velocity.X * velocity.X + velocity.Y * velocity.Y + velocity.Z * velocity.Z);
        var angular = description.Velocity.Angular;
        var absoluteAngular = Math.Sqrt(angular.X * angular.X + angular.Y * angular.Y + angular.Z * angular.Z);
        var message =
            $"Velocity: ({velocity.X:F2}, {velocity.Y:F2}, {velocity.Z:F2}), absolute velocity: {absoluteVelocity:F2}, " +
            $"angunar: ({angular.X:F2}, {angular.Y:F2}, {angular.Z:F2}), absolute angular: {absoluteAngular:F2}.";

        var bottomY = renderer.Surface.Resolution.Y;
        renderer.TextBatcher.Write(
            text.Clear().Append(message),
            new Vector2(16, bottomY - 64), 16, Vector3.One, font);
    }

    private void OutputOrbiterStatistics(Renderer renderer, TextBuilder text, Font font)
    {
        if (!(hasOrbiters || hasMolecules))
        {
            return;
        }

        var bodyHandles = hasOrbiters ? orbiterHandles : moleculeHandles;
        orbiterStatisticsCallCount++;

        // Only calculate statistics every N calls
        if (orbiterStatisticsCallCount % orbiterStatisticsReportingFrequency == 0)
        {
            CalculateOrbiterStatistics(bodyHandles);
            CollectStatistics();
        }

        DisplayOrbiterStatistics(renderer, text, font);
    }

    private void CalculateOrbiterStatistics(BodyHandle[] bodyHandles)
    {
        float totalSpeedX = 0, totalSpeedY = 0, totalSpeedZ = 0;
        float totalAbsoluteSpeed = 0;
        float totalAngularX = 0, totalAngularY = 0, totalAngularZ = 0;
        float totalAbsoluteAngular = 0;
        float totalPositionX = 0, totalPositionY = 0, totalPositionZ = 0;
        float totalAbsolutePosition = 0;
        float totalKineticEnergy = 0;
        float totalPotentialEnergy = 0;
        float minAbsolutePosition = float.MaxValue;
        float maxAbsolutePosition = float.MinValue;
        int insidePlanetCount = 0;
        int runawayCount = 0;
        float totalAngularMomentumX = 0, totalAngularMomentumY = 0, totalAngularMomentumZ = 0;

        var orbiterCount = bodyHandles.Length;

        foreach (var handle in bodyHandles)
        {
            Simulation.Bodies.GetDescription(handle, out var description);

            var velocity = description.Velocity.Linear;
            var angular = description.Velocity.Angular;
            var position = description.Pose.Position;

            totalSpeedX += velocity.X;
            totalSpeedY += velocity.Y;
            totalSpeedZ += velocity.Z;
            float absoluteSpeed =
                (float)Math.Sqrt(velocity.X * velocity.X + velocity.Y * velocity.Y + velocity.Z * velocity.Z);
            totalAbsoluteSpeed += absoluteSpeed;

            totalAngularX += angular.X;
            totalAngularY += angular.Y;
            totalAngularZ += angular.Z;
            totalAbsoluteAngular +=
                (float)Math.Sqrt(angular.X * angular.X + angular.Y * angular.Y + angular.Z * angular.Z);

            totalPositionX += position.X;
            totalPositionY += position.Y;
            totalPositionZ += position.Z;
            float absolutePosition =
                (float)Math.Sqrt(position.X * position.X + position.Y * position.Y + position.Z * position.Z);
            totalAbsolutePosition += absolutePosition;

            // Energy calculations
            float kineticEnergy = (absoluteSpeed * absoluteSpeed) / 2.0f;
            totalKineticEnergy += kineticEnergy;

            float potentialEnergy = (-gravityValue) / absolutePosition;
            totalPotentialEnergy += potentialEnergy;

            // Track minimum and maximum absolute position
            if (absolutePosition < minAbsolutePosition)
            {
                minAbsolutePosition = absolutePosition;
            }

            if (absolutePosition > maxAbsolutePosition)
            {
                maxAbsolutePosition = absolutePosition;
            }

            // Count orbiters inside planet
            if (absolutePosition < PlanetRadius - orbiterRadius)
            {
                insidePlanetCount++;
            }

            // Count runaway orbiters (where kinetic energy exceeds escape energy)
            float escapeEnergy = (absoluteSpeed * absoluteSpeed) / 2.0f - gravityValue / absolutePosition;
            if (escapeEnergy > 0)
            {
                runawayCount++;
            }

            // Angular momentum calculation: L = r × v (cross product)
            Vector3 angularMomentum = Vector3.Cross(position, velocity);
            totalAngularMomentumX += angularMomentum.X;
            totalAngularMomentumY += angularMomentum.Y;
            totalAngularMomentumZ += angularMomentum.Z;
        }

        // Calculate averages
        averageSpeed = new Vector3(totalSpeedX / orbiterCount, totalSpeedY / orbiterCount, totalSpeedZ / orbiterCount);
        averageAbsoluteSpeed = totalAbsoluteSpeed / orbiterCount;

        averageAngularSpeed = new Vector3(totalAngularX / orbiterCount, totalAngularY / orbiterCount,
            totalAngularZ / orbiterCount);
        averageAbsoluteAngularSpeed = totalAbsoluteAngular / orbiterCount;

        averagePosition = new Vector3(totalPositionX / orbiterCount, totalPositionY / orbiterCount,
            totalPositionZ / orbiterCount);
        averageAbsolutePosition = totalAbsolutePosition / orbiterCount;

        averageKineticEnergy = totalKineticEnergy / orbiterCount;
        averagePotentialEnergy = totalPotentialEnergy / orbiterCount;
        averageTotalEnergy = averageKineticEnergy + averagePotentialEnergy;
        minimumAbsolutePosition = minAbsolutePosition;
        maximumAbsolutePosition = maxAbsolutePosition;

        orbitersInsidePlanet = insidePlanetCount;
        runawayOrbiters = runawayCount;
        averageAngularMomentum = new Vector3(totalAngularMomentumX / orbiterCount, totalAngularMomentumY / orbiterCount,
            totalAngularMomentumZ / orbiterCount);
        averageAbsoluteAngularMomentum = (float)Math.Sqrt(averageAngularMomentum.X * averageAngularMomentum.X +
                                                          averageAngularMomentum.Y * averageAngularMomentum.Y +
                                                          averageAngularMomentum.Z * averageAngularMomentum.Z);
    }

    private void DisplayOrbiterStatistics(Renderer renderer, TextBuilder text, Font font)
    {
        var bottomY = renderer.Surface.Resolution.Y;

        var message1 =
            $"Orbiters (call {orbiterStatisticsCallCount}): avg speed: ({averageSpeed.X:F2}, {averageSpeed.Y:F2}, {averageSpeed.Z:F2}), " +
            $"avg absolute speed: {averageAbsoluteSpeed:F2}, " +
            $"avg angular: ({averageAngularSpeed.X:F2}, {averageAngularSpeed.Y:F2}, {averageAngularSpeed.Z:F2}), avg absolute angular: {averageAbsoluteAngularSpeed:F2}, " +
            $"avg position: ({averagePosition.X:F2}, {averagePosition.Y:F2}, {averagePosition.Z:F2}), avg absolute position: {averageAbsolutePosition:F2}.";

        var message2 =
            $"Energy: avg kinetic: {averageKineticEnergy:F2}, avg potential: {averagePotentialEnergy:F2}, avg total: {averageTotalEnergy:F2}, " +
            $"min absolute position: {minimumAbsolutePosition:F2}, max absolute position: {maximumAbsolutePosition:F2}, inside planet: {orbitersInsidePlanet}, runaway: {runawayOrbiters}, " +
            $"avg angular momentum: ({averageAngularMomentum.X:F2}, {averageAngularMomentum.Y:F2}, {averageAngularMomentum.Z:F2}), abs: {averageAbsoluteAngularMomentum:F2}.";

        renderer.TextBatcher.Write(
            text.Clear().Append(message1),
            new Vector2(16, bottomY - 80), 16, Vector3.One, font);

        renderer.TextBatcher.Write(
            text.Clear().Append(message2),
            new Vector2(16, bottomY - 96), 16, Vector3.One, font);
    }

    #endregion

    protected override void OnDispose()
    {
        ExportToCsv(prefix, exportFolder);
    }
}
