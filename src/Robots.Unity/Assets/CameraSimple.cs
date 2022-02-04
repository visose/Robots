using UnityEngine;
using UnityEngine.InputSystem;

namespace Robots.Unity
{
    public class CameraSimple : MonoBehaviour
    {
        Vector3 _target = new(0, 0.5f, 0);

        void Update()
        {
            var mouse = Mouse.current;
            var shouldRotate = mouse.rightButton.isPressed;

            if (shouldRotate)
            {
                var delta = mouse.delta.ReadValue() * 0.2f;

                transform.RotateAround(_target, Vector3.up, delta.x);
                transform.RotateAround(_target, transform.rotation * Vector3.right, -delta.y);
            }

            var scroll = mouse.scroll.ReadValue();
            var shouldZoom = scroll.y != 0;

            if (shouldZoom)
            {
                float delta = Mathf.Sign(scroll.y) * 0.1f;
                float distance = (_target - transform.position).magnitude * delta;
                transform.Translate(Vector3.forward * distance, Space.Self);
            }
        }
    }
}