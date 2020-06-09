//
// Angle Compensator for Nav Scanner
//

#ifndef SICK_SCAN_ANGLE_COMPENSATOR_H
#define SICK_SCAN_ANGLE_COMPENSATOR_H

#include <string>
#include <vector>

class AngleCompensator
{
public:
  double compensateAngleInRad(double angleInRad);
  double compensateAngleInDeg(double angleInDeg);
  int parseAsciiReply(const char *asciiReply);
  int parseReply(bool isBinary, std::vector<unsigned char>& replyVec);
  void testbed();
private:

  double amplCorr;
  double phaseCorrInDeg;
  double offsetCorrInDeg;
  double phaseCorrInRad;
  double offsetCorrInRad;

};


#endif //SICK_SCAN_ANGLE_COMPENSATOR_H
