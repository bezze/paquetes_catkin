#include <tf/transform_listener.h>

namespace tf
{

bool lookupTransformSafe(const tf::TransformListener& listener,
                         const std::string& targetFrame,
                         const std::string& sourceFrame,
                         const ros::Time& time,
                         tf::StampedTransform& sourceToTarget)
{
  bool retVal = true;

  // First try to transform the data at the requested time
  try
  {
    listener.lookupTransform(targetFrame, sourceFrame, time, sourceToTarget);
  }
  catch (tf::TransformException& ex)
  {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    try
    {
      listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), sourceToTarget);

      ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                    " was unavailable for the time requested. Using latest instead.\n");
    }
    catch(tf::TransformException& ex)
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << sourceFrame <<
                                    " to " << targetFrame << ". Error was " << ex.what() << "\n");

      retVal = false;
    }
  }

  // Transforming from a frame id to itself can fail when the tf tree isn't
  // being broadcast (e.g., for some bag files). This is the only failure that
  // would throw an exception, so check for this situation before giving up.
  if (not retVal)
  {
    if (targetFrame == sourceFrame)
    {
      sourceToTarget.setIdentity();
      retVal = true;
    }
  }

  return retVal;
}
 
} // tf
