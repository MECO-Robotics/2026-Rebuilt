package frc.robot.systemcheck;

/** Result state for one automated robot check. */
public enum CheckStatus {
	NOT_RUN, RUNNING, PASS, WARNING, FAIL, SKIPPED, AWAITING_CONFIRMATION, ABORTED
}
