using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;
using DemoRenderer.ShapeDrawing;

namespace Demos.Demos;


/// <summary>
/// Shows how to use custom velocity integration to implement planetary gravity.
/// </summary>
public class PlanetNonRotatingMoleculesDemo : Demo
{
    #region Physical Parameters

    private float gravityValue;

    // float frequency = 30.0f;
    // float dampingRatio = 0.0f;

    float maximumRecoveryVelocity = float.MaxValue;
    float frictionCoefficient = 0.0f;

    float thickness = 5.0f;
    int numberOfLayers = 1;

    #region velocityIterationCount: 8, substepCount: 1, frequency: 5.0

    int velocityIterationCount = 8;
    int substepCount = 1;
    float frequency = 5.0f;
    float dampingRatio = -0.2625f;

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

    #region Parameters

    static float testVelocityValue = 0f;

    static float testOriginValue = 500f;
    Vector3 testOrigin = new Vector3(-testOriginValue, 0, 0);
    Vector3 testOrigin2 = new Vector3(testOriginValue, 0, 0);
    Vector3 testVelocity = new Vector3(testVelocityValue, 0, 0);
    Vector3 testVelocity2 = new Vector3(-testVelocityValue, 0, 0);

    float PlanetRadius = 50.0f;
    Vector3 PlanetCenter = new Vector3();
    int subDivisionSteps = 8;

    float orbiterRadius = 1.0f;
    float orbiterMass = 1.0f;

    float moleculeRadius = 0.5f;
    float moleculeMass = 0.125f;

    // const int count = 40;
    const int count = 40;

    const int length = count;
    const int width = count;
    const int height = count;

    private const int planetMeshWidth = 20;

    Vector3 mainOrigin = new Vector3(-200, 300, 0);
    Vector3 mainMoleculeOrigin = new Vector3();

    Vector3 mainVelocity = new Vector3();
    Vector3 mainMoleculeVelocity = new Vector3(0, 0, 0);

    Vector3 spacing = new Vector3(5);

    StaticHandle PlanetHandle;

    Vector3 position = new Vector3(0, 0, 0);
    Quaternion rotation = QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2);
    Vector3 scalingVector = new Vector3(1, 1, 1);

    private BodyHandle testHandle;
    private bool hasTestOrbiter;

    private BodyHandle[] orbiterHandles;
    private bool hasOrbiters;

    # region Statistics

    private int orbiterStatisticsCallCount = -1;
    private int orbiterStatisticsReportingFrequency = 10;
    private Vector3 averageSpeed = Vector3.Zero;
    private float averageAbsoluteSpeed = 0;
    private Vector3 averageAngularSpeed = Vector3.Zero;
    private float averageAbsoluteAngularSpeed = 0;
    private Vector3 averagePosition = Vector3.Zero;
    private float averageAbsolutePosition = 0;
    private float averageKineticEnergy = 0;
    private float averagePotentialEnergy = 0;
    private float averageTotalEnergy = 0;
    private float minimumAbsolutePosition = 0;
    private float maximumAbsolutePosition = 0;
    private int orbitersInsidePlanet = 0;
    private Vector3 averageAngularMomentum = Vector3.Zero;
    private float averageAbsoluteAngularMomentum = 0;

    #endregion

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

    #region Helper Methods

    private void SetCamera(Camera camera)
    {
        camera.Position = new Vector3(-110, 80, -50);
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
        // Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Cylinder(20, 100))));

        // var sphereShape = new Sphere(50);
        // var material = new DefaultMaterial { Color = new Vector4(1, 1, 1, 0.5f) }; // RGBA: A=0.5 for 50% transparency
        // Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(sphereShape), material));
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
        var molecule = new Sphere(moleculeRadius);
        var moleculeInertia = molecule.ComputeInertia(moleculeMass);
        var moleculeShapeIndex = Simulation.Shapes.Add(molecule);

        for (var i = 0; i < length; ++i)
        {
            for (var j = 0; j < height; ++j)
            {
                var origin = mainMoleculeOrigin + spacing * new Vector3(length * -0.5f, 0, width * -0.5f);
                for (var k = 0; k < width; ++k)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                        origin + new Vector3(i, j, k) * spacing, mainMoleculeVelocity, moleculeInertia,
                        moleculeShapeIndex, activity));
                }
            }
        }
    }

    #endregion

    #region Initialize

    public override void Initialize(ContentArchive content, Camera camera)
    {
        SetGravity();
        CreateSimulation(camera);

        #region Planet

        CreatePlanet();
        // CreateMeshCylinder();
        // CreateMeshSphere();

        // CreateThickMeshSphere();
        // CreateThickPlate();

        #endregion

        CreateTestOrbiter();
        // CreateTestOrbiter2();

        CreateOrbiters();
        // CreateMolecules();
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
            text.Clear().Append("In this demo, all bodies are pulled towards the center of the planet."),
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
        if (!hasOrbiters)
        {
            return;
        }

        orbiterStatisticsCallCount++;

        // Only calculate statistics every N calls
        if (orbiterStatisticsCallCount % orbiterStatisticsReportingFrequency == 0)
        {
            CalculateOrbiterStatistics();
        }

        DisplayOrbiterStatistics(renderer, text, font);
    }

    private void CalculateOrbiterStatistics()
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
        float totalAngularMomentumX = 0, totalAngularMomentumY = 0, totalAngularMomentumZ = 0;

        var orbiterCount = orbiterHandles.Length;

        foreach (var handle in orbiterHandles)
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
            $"min absolute position: {minimumAbsolutePosition:F2}, max absolute position: {maximumAbsolutePosition:F2}, inside planet: {orbitersInsidePlanet}, " +
            $"avg angular momentum: ({averageAngularMomentum.X:F2}, {averageAngularMomentum.Y:F2}, {averageAngularMomentum.Z:F2}), abs: {averageAbsoluteAngularMomentum:F2}.";

        renderer.TextBatcher.Write(
            text.Clear().Append(message1),
            new Vector2(16, bottomY - 80), 16, Vector3.One, font);

        renderer.TextBatcher.Write(
            text.Clear().Append(message2),
            new Vector2(16, bottomY - 96), 16, Vector3.One, font);
    }

    #endregion
}
