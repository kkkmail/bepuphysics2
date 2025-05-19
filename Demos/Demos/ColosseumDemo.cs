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

namespace Demos.Demos;

/// <summary>
/// A colosseum made out of boxes that is sometimes hit by large purple hail.
/// </summary>
public class ColosseumDemo : Demo
{
    public static void CreateRingWall(Simulation simulation, Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, int height, float radius)
    {
        var circumference = MathF.PI * 2 * radius;
        var boxCountPerRing = (int)(0.9f * circumference / ringBoxShape.Length);
        float increment = MathHelper.TwoPi / boxCountPerRing;
        for (int ringIndex = 0; ringIndex < height; ringIndex++)
        {
            for (int i = 0; i < boxCountPerRing; i++)
            {
                var angle = ((ringIndex & 1) == 0 ? i + 0.5f : i) * increment;
                bodyDescription.Pose = (position + new Vector3(-MathF.Cos(angle) * radius, (ringIndex + 0.5f) * ringBoxShape.Height, MathF.Sin(angle) * radius), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle));
                simulation.Bodies.Add(bodyDescription);
            }
        }
    }

    public static void CreateRingPlatform(Simulation simulation, Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, float radius)
    {
        var innerCircumference = MathF.PI * 2 * (radius - ringBoxShape.HalfLength);
        var boxCount = (int)(0.95f * innerCircumference / ringBoxShape.Height);
        float increment = MathHelper.TwoPi / boxCount;
        for (int i = 0; i < boxCount; i++)
        {
            var angle = i * increment;
            bodyDescription.Pose = (
                position + new Vector3(-MathF.Cos(angle) * radius, ringBoxShape.HalfWidth, MathF.Sin(angle) * radius),
                QuaternionEx.Concatenate(QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f),
                    QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle + MathF.PI * 0.5f)));
            simulation.Bodies.Add(bodyDescription);
        }
    }

    public static Vector3 CreateRing(Simulation simulation, Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, float radius, int heightPerPlatformLevel, int platformLevels)
    {
        for (int platformIndex = 0; platformIndex < platformLevels; ++platformIndex)
        {
            var wallOffset = ringBoxShape.HalfLength - ringBoxShape.HalfWidth;
            CreateRingWall(simulation, position, ringBoxShape, bodyDescription, heightPerPlatformLevel, radius + wallOffset);
            CreateRingWall(simulation, position, ringBoxShape, bodyDescription, heightPerPlatformLevel, radius - wallOffset);
            CreateRingPlatform(simulation, position + new Vector3(0, heightPerPlatformLevel * ringBoxShape.Height, 0), ringBoxShape, bodyDescription, radius);
            position.Y += heightPerPlatformLevel * ringBoxShape.Height + ringBoxShape.Width;
        }

        return position;
    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        // camera.Position = new Vector3(-30, 40, -30);
        // camera.Yaw = MathHelper.Pi * 3f / 4;
        // camera.Pitch = MathHelper.Pi * 0.2f;

        // camera.Position = new Vector3(110, -80, 12);
        // camera.Yaw = 0;
        // camera.Pitch = MathF.PI * -0.5f;

        // camera.Position = new Vector3(110, 80, 12);
        // camera.Yaw = 0;
        // camera.Pitch = MathF.PI * -0.5f;

        // camera.Position = new Vector3(-110, -80, -50);
        camera.Position = new Vector3(-110, 80, -50);
        camera.Yaw = 0;
        camera.Pitch = MathF.PI * -0.5f;

        // camera.Position = new Vector3(0, 10000, 0);
        // camera.Yaw = 0;
        // camera.Pitch = MathF.PI;

        // camera.Position = new Vector3(0, 200, 0);
        // camera.Yaw = 0;
        // camera.Pitch = MathF.PI;

        // More or less OK, but get under the surface.
        // var frequency = 5.0f;
        // var dampingRatio = -0.25f;
        // var orbiterRadius = 0.3f;
        // var orbiterMass = 1.0f;

        // var frequency = 30.0f;
        // var dampingRatio = -5.0f;
        // var orbiterRadius = 0.3f;
        // var orbiterMass = 1.0f;

        // Anti-damping is not enough.
        // var frequency = 30.0f;
        // var dampingRatio = -3.0f;
        // var orbiterRadius = 0.3f;
        // var orbiterMass = 100.0f;

        // =================================================
        var gravityValue = 100000.0f;

        // var frictionCoefficient = 0.0f;

        // Seems OK but over anti-damped. 0.25 is not enough.
        // var frequency = 5.0f;
        // var dampingRatio = -0.30f;
        // var orbiterRadius = 1.0f;
        // var orbiterMass = 1.0f;

        // var frequency = 5.0f;
        // var dampingRatio = -0.275f;
        // var orbiterRadius = 1.0f;
        // var orbiterMass = 1.0f;

        // Stable enough but loses molecules due to boost on collision with the planet.
        // var frequency = 5.0f;
        // var dampingRatio = -0.265f;
        // var orbiterRadius = 1.0f;
        // var orbiterMass = 1.0f;

        // WTF?
        // var frequency = 5.0f;
        // var dampingRatio = -0.26f;
        // var orbiterRadius = 1.0f;
        // var orbiterMass = 1.0f;

        // Deep freeze.
        // var frequency = 5.0f;
        // var dampingRatio = -0.25f;
        // var orbiterRadius = 1.0f;
        // var orbiterMass = 1.0f;

        // Good enough.
        // var frequency = 5.0f;
        // var dampingRatio = -0.2625f;
        // var orbiterRadius = 1.0f;
        // var orbiterMass = 1.0f;

        // =================================================

        // Stable
        var frictionCoefficient = 1.0f;
        var frequency = 5.0f;
        var dampingRatio = -0.375f;
        var orbiterRadius = 1.0f;
        var orbiterMass = 1.0f;

        // Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 0)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
        // Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 0)), new DemoPoseIntegratorCallbacks(new Vector3()), new SolveDescription(8, 1));
        // Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 0)), new DemoPoseIntegratorCallbacks(new Vector3()), new SolveDescription(8, 1));
        Simulation = Simulation.Create(BufferPool,
            new DemoNarrowPhaseCallbacks(
                new SpringSettings(frequency, dampingRatio),
                maximumRecoveryVelocity: float.MaxValue,
                frictionCoefficient: frictionCoefficient),
            new DemoPoseIntegratorCallbacks(
                new Vector3(),
                linearDamping: 0f,
                angularDamping: 0f,
                gravityValue: gravityValue),
            new SolveDescription(8, 1));

        // var ringBoxShape = new Box(0.5f, 1, 3);
        // var boxDescription = BodyDescription.CreateDynamic(new Vector3(), ringBoxShape.ComputeInertia(1), Simulation.Shapes.Add(ringBoxShape), 0.01f);

        // var layerPosition = new Vector3();
        // const int layerCount = 6;
        // var innerRadius = 15f;
        // var heightPerPlatform = 3;
        // var platformsPerLayer = 1;
        // var ringSpacing = 0.5f;

        // for (int layerIndex = 0; layerIndex < layerCount; ++layerIndex)
        // {
        //     var ringCount = layerCount - layerIndex;
        //     for (int ringIndex = 0; ringIndex < ringCount; ++ringIndex)
        //     {
        //         CreateRing(Simulation, layerPosition, ringBoxShape, boxDescription, innerRadius + ringIndex * (ringBoxShape.Length + ringSpacing) + layerIndex * (ringBoxShape.Length - ringBoxShape.Width), heightPerPlatform, platformsPerLayer);
        //     }
        //     layerPosition.Y += platformsPerLayer * (ringBoxShape.Height * heightPerPlatform + ringBoxShape.Width);
        // }

        //Console.WriteLine($"box count: {Simulation.Bodies.ActiveSet.Count}");
        // Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(500, 1, 500))));

        #region Planet

        var planet = new Sphere(50);
        Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(planet)));

        #endregion

        var bulletShape = new Sphere(0.5f);
        bulletDescription = BodyDescription.CreateDynamic(new Vector3(), bulletShape.ComputeInertia(.1f), Simulation.Shapes.Add(bulletShape), 0.01f);

        var shootiePatootieShape = new Sphere(3f);
        shootiePatootieDescription = BodyDescription.CreateDynamic(new Vector3(), shootiePatootieShape.ComputeInertia(100), new(Simulation.Shapes.Add(shootiePatootieShape), 0.1f), 0.01f);

        var orbiterShape = new Sphere(orbiterRadius);
        // var orbiterShape = new Sphere(100);
        var orbiterDescription = BodyDescription.CreateDynamic(new Vector3(), bulletShape.ComputeInertia(orbiterMass), Simulation.Shapes.Add(orbiterShape), 0.01f);

        var inertia = orbiterShape.ComputeInertia(orbiterMass);
        var orbiterShapeIndex = Simulation.Shapes.Add(orbiterShape);
        var spacing = new Vector3(5);
        const int length = 40;
        // const int length = 1;

        const int depth = 40;
        // const int depth = 20;
        for (int i = 0; i < length; ++i)
        {
            for (int j = 0; j < depth; ++j)
            {
                const int width = 40;
                // const int width = 1;
                var origin = new Vector3(-200, 300, 0) + spacing * new Vector3(length * -0.5f, 0, width * -0.5f);
                for (int k = 0; k < width; ++k)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(origin + new Vector3(i, j, k) * spacing, new Vector3(), inertia, orbiterShapeIndex, 0.01f));
                    // orbiterDescription.Pose.Position = origin + new Vector3(i, j, k) * spacing;
                    // // orbiterDescription.Velocity.Linear = new Vector3(0, -20, 0);
                    // Simulation.Bodies.Add(orbiterDescription);
                }
            }
        }

        // var xPosition = 10000;
        // var xVelocity = 1000;
        // Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(-xPosition, 0, 0), new Vector3(xVelocity, 0, 0), inertia, orbiterShapeIndex, 0.01f));
        // Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(xPosition, 0, 0), new Vector3(-xVelocity, 0, 0), inertia, orbiterShapeIndex, 0.01f));
    }

    BodyDescription bulletDescription;
    BodyDescription shootiePatootieDescription;

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        if (input != null)
        {
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                bulletDescription.Pose.Position = camera.Position;
                bulletDescription.Velocity.Linear = camera.GetRayDirection(input.MouseLocked, window.GetNormalizedMousePosition(input.MousePosition)) * 400;
                Simulation.Bodies.Add(bulletDescription);
            }
            else if (input.WasPushed(OpenTK.Input.Key.X))
            {
                shootiePatootieDescription.Pose.Position = camera.Position;
                shootiePatootieDescription.Velocity.Linear = camera.GetRayDirection(input.MouseLocked, window.GetNormalizedMousePosition(input.MousePosition)) * 100;
                Simulation.Bodies.Add(shootiePatootieDescription);
            }
        }

        base.Update(window, camera, input, dt);
    }

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        text.Clear().Append("Press Z to shoot a bullet, press X to super shootie patootie!");
        renderer.TextBatcher.Write(text, new Vector2(20, renderer.Surface.Resolution.Y - 20), 16, new Vector3(1, 1, 1), font);
        base.Render(renderer, camera, input, text, font);
    }
}
