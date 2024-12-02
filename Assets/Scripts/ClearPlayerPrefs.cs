using UnityEngine;

public class ClearPlayerPrefs : MonoBehaviour
{
    void Start()
    {
        PlayerPrefs.DeleteKey("SphereRadius");
        Debug.Log("PlayerPrefs for SphereRadius has been reset.");
    }
}
