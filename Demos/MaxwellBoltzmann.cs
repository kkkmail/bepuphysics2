using System;
using System.Numerics;

namespace Demos;

public static class MaxwellBoltzmann
{

    /// <summary>
    /// Generates a random 3D velocity vector following the Maxwell–Boltzmann distribution
    /// 'rng' is the random number generator
    /// 'v' is the root-mean-square speed (v^2 = average(v^2))
    /// </summary>
    public static Vector3 NextVelocity(this Random rng, double v)
    {
        // Standard deviation for each component
        var sigma = v / Math.Sqrt(3);

        // Generate three independent standard normal variables using Box–Muller transform
        var x = sigma * NextStandardNormal(rng);
        var y = sigma * NextStandardNormal(rng);
        var z = sigma * NextStandardNormal(rng);

        return new Vector3((float)x, (float)y, (float)z);
    }

    /// <summary>
    /// Generates a standard normal distributed value using the Box–Muller transform
    /// </summary>
    private static double NextStandardNormal(Random rng)
    {
        var u1 = 1.0 - rng.NextDouble(); // Avoid zero
        var u2 = 1.0 - rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }
}