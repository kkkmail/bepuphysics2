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
    #region Parameters

    float frequency = 5.0f;
    float dampingRatio = -0.2625f;

    float maximumRecoveryVelocity = float.MaxValue;
    float frictionCoefficient = 0.0f;

    float PlanetRadius = 50.0f;
    private Vector3 PlanetCenter = new Vector3();
    int subDivisionSteps = 8;

    float gravityValue = 100000.0f;

    float orbiterRadius = 1.0f;
    float orbiterMass = 1.0f;

    float moleculeRadius = 0.5f;
    float moleculeMass = 1.0f;

    const int count = 40;
    // const int count = 5;

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

    #endregion

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
            // var offset = position - Vector3Wide.Broadcast(PlanetCenter);
            // var distance = offset.Length();
            //
            // if (distance[0] >= Radius)
            // {
            //     velocity.Linear -= new Vector<float>(gravityDt) * offset /
            //                        Vector.Max(Vector<float>.One, distance * distance * distance);
            // }
        }
    }

    private void SetCamera(Camera camera)
    {
        camera.Position = new Vector3(-110, 80, -50);
        camera.Yaw = 0;
        camera.Pitch = MathF.PI * -0.5f;
    }

    private void CreateSimulation()
    {
        Simulation = Simulation.Create(BufferPool,
            new DemoNarrowPhaseCallbacks(
                new SpringSettings(frequency, dampingRatio),
                maximumRecoveryVelocity: maximumRecoveryVelocity,
                frictionCoefficient: frictionCoefficient),
            new PlanetaryGravityCallbacks
                { PlanetCenter = PlanetCenter, Gravity = gravityValue, Radius = PlanetRadius },
            new SolveDescription(8, 1));
    }

    private void CreatePlanet()
    {
        PlanetHandle = Simulation.Statics.Add(new StaticDescription(PlanetCenter, Simulation.Shapes.Add(new Sphere(PlanetRadius))));
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
        var position = new Vector3(0, 0, 0);
        var rotation = QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2);
        var scalingVector = new Vector3(1, 1, 1);

        var planetMesh = PlanetMeshCreator.CreatePlanetMesh(PlanetRadius, subDivisionSteps, scalingVector, BufferPool, ThreadDispatcher);
        Simulation.Statics.Add(new StaticDescription(position, rotation, Simulation.Shapes.Add(planetMesh)));
    }

    private void CreateOrbiters()
    {
        var orbiter = new Sphere(orbiterRadius);
        var inertia = orbiter.ComputeInertia(orbiterMass);
        var orbiterShapeIndex = Simulation.Shapes.Add(orbiter);

        for (var i = 0; i < length; ++i)
        {
            for (var j = 0; j < height; ++j)
            {
                var origin = mainOrigin + spacing * new Vector3(length * -0.5f, 0, width * -0.5f);
                for (var k = 0; k < width; ++k)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                        origin + new Vector3(i, j, k) * spacing, mainVelocity, inertia, orbiterShapeIndex, 0.01f));
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
                        origin + new Vector3(i, j, k) * spacing, mainMoleculeVelocity, moleculeInertia, moleculeShapeIndex, 0.01f));
                }
            }
        }
    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        SetCamera(camera);
        CreateSimulation();
        // CreatePlanet();
        // CreateMeshCylinder();
        CreateMeshSphere();
        CreateOrbiters();
        CreateMolecules();
    }

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
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
}
