using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.Universal;
using UnityEditor;
using Unity.Mathematics;
public class EditorDebugCameraBehaviour : MonoBehaviour
{
    [SerializeField] Camera _cameraToOverlay;
    void Start()
    {
        Camera cameraOfThisObject = gameObject.GetComponent<Camera>();
        if(cameraOfThisObject == null) { return; }
        _cameraToOverlay.GetComponent<UniversalAdditionalCameraData>().cameraStack.Add(cameraOfThisObject);
    }

    void Update()
    {
        transform.position = _cameraToOverlay.transform.position;
        transform.rotation = _cameraToOverlay.transform.rotation;
        transform.localScale = _cameraToOverlay.transform.localScale;
    }
}
