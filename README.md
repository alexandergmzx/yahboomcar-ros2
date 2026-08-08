# yahboomcar-ros2 — robot1 first-party stack (extracted, history preserved)

Alex's authorship from the MicroROS working repo, extracted via
git-filter-repo (fleet D-12, session 4): safety, localization, simulators,
twin, tools, measured docs, and `yahboomcar_config` (corrected EKF + tuned
SLAM + robot1's canonical SLAM launch, D-19). 63 commits of history follow
every file (`git log --follow` verified at extraction).

Vendor runtime + course archive: the MicroROS checkout, permanently
(unlicensed vendor code, R-05 — never republished here). Read
[`CLAUDE.md`](CLAUDE.md) for the firmware contract and hard constraints;
the fleet's decision tables live in the robot-fleet repo.
