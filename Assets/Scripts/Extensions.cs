using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts
{
    public static class QuaternionExtensions
    {
        public static Quaternion ScalarMultiply(this Quaternion input, float scalar)
        {
            return new Quaternion(input.x * scalar, input.y * scalar, input.z * scalar, input.w * scalar);
        }

        public static Quaternion Multiply(this Quaternion input, Vector3 vector)
        {
            return new Quaternion(input.x * vector.x, input.y * vector.y, input.z * vector.z, input.w);
        }
    }
}
