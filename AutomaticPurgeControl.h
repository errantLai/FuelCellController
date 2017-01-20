
#ifndef AutomaticPurgeControl
#define AutomaticPurgeControl

class AutomaticPurgeControl {
public:
	AutomaticPurgeControl();
	// Determines if the purge valve should be opened, based on given conditions
	bool OpenPurgeValve(bool purgeValveOpen, bool fcRelayOpen, float fcCurrent);
	int PURGE_DURATION;
private:
	float purgeLastCallTime;
	float AmpSecSincePurge;
	int PurgeOpenCounter;
};

#endif
