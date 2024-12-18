using UnityEngine;
using RosMessageTypes.JointControl;

public static class JointDataManager
{
    private static Joint_listMsg savedJointMessage;
    private static bool isJointMessageSaved = false;

    public static void SetJointMessage(Joint_listMsg message)
    {
        savedJointMessage = message;
        isJointMessageSaved = true;
        Debug.Log("Joint message saved");
    }

    public static Joint_listMsg GetSavedJointMessage()
    {
        return savedJointMessage;
    }

    public static bool IsJointMessageSaved()
    {
        return isJointMessageSaved;
    }
}
