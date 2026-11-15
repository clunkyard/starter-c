/* A robot on the Clunkyard core's C API. It searches with a broad scan,
 * narrows the beam on a contact, and leads shots from a line through its
 * recent sightings. Passive cues guide its search. The core owns tracking
 * and shot geometry.
 * Clunkyard robot API 2.0.0: one core under every language, the same
 * answers everywhere. Facts (a call shows its arguments):
 * match: mode ruleset control robots is_3d tick_rate max_ticks
 *   robot_id(slot)
 * clock: tick seconds remaining_ticks
 * arena: size gravity terrain_kind ground_height(x y)
 * me: health position velocity heading pitch roll angular_velocity
 *   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
 *   from_mount(heading pitch) mount_toward(point)
 * battery: charge capacity regen sustainable_drive compute
 *   drive_cost(fraction)
 * drive: hover_lift
 * gun: aim_now cooldown_ticks spread max_power damage_scale
 *   can_fire(power reserve) shot_speed(power) shot_cost(power)
 *   reload_ticks(power) shot_damage(power) power_for_speed(speed)
 *   power_for_flight_time(range seconds) lead(contact power)
 *   lead_point(position velocity power)
 * scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
 * sensors: contacts nearest cues contact(slot) position_of(contact)
 * tracker: cue nearest_bearing_rate velocity(slot) position(slot)
 *   history(slot) last_seen(slot)
 * nav: zone zone.margin(point) zone.contains(point margin)
 *   zone.distance_from(point) storm storm.margin(point)
 *   storm.contains(point margin) storm.distance_from(point) hill
 *   hill.margin(point) hill.contains(point margin)
 *   hill.distance_from(point) pickups nearest_pickup(kind)
 *   field_distance(a b)
 * score: me best armour
 * angles: normalize(degrees) angle_diff(target current)
 *   bearing_to(from to) elevation_to(from to) direction(heading pitch)
 *   distance(a b) to_chassis(heading pitch attitude)
 *   from_chassis(heading pitch attitude) clamp(value low high)
 * raw: number(path) text(path) has(path) flag(path) count(path) line()
 * Controls: drive.set(throttle turn pitch_turn)
 *   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
 *   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
 *   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
 *   status.set(text value) raw.extra(json)
 * Every control argument is optional; a second call keeps what the first
 * set. Absent facts are your language's none. Names are snake_case here;
 * your language cases them its own way. Angles are degrees from the +x
 * axis; pitch is positive up.
 * Manual: https://clunkyard.com/manual#sdk
 */
#include "clunkyard.h"
#include <math.h>
#include <stdint.h>

static double get(cy_robot *robot, int32_t (*query)(cy_robot *, double *)) {
  double value=0; query(robot,&value); return value;
}
static double steer(double towards,double current) {
  double delta=0,value=0;cy_angles_angle_diff(towards,current,&delta);
  cy_angles_clamp(delta/45,-1,1,&value);return value;
}
static void drive(cy_robot *robot,double throttle,double turn,double pitch) {
  cy_drive_throttle(robot,throttle);cy_drive_turn(robot,turn);cy_drive_pitch_turn(robot,pitch);
}
static void scan(cy_robot *robot,double heading,double arc,double pitch) {
  cy_scanner_heading(robot,heading);cy_scanner_arc(robot,arc);cy_scanner_pitch(robot,pitch);
}
/* Bearing is earned: a scan contact's bearing is off by up to its bearing_error
 * (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
 * one aims at it. Contacts within 120 units are exact. */
#define SEARCH_ARC 40.0 /* degrees: sweeps the circle in half a second */
#define HOLD_WIDTH 60.0 /* units across at the target: it stays in the beam from tick to tick */
#define MIN_ARC 6.0     /* degrees: the narrowest the beam holds */
#define WINDOW 12       /* ticks of sightings the aim averages */
#define SIGHTINGS 8     /* narrow sightings in the window before a shot */
#define LOOK_EVERY 40   /* ticks between looks around the target, for a nearer robot */
#define REACQUIRE 10    /* ticks to look again where a lost target was */
/* A straight line through the narrowest of the last WINDOW ticks' sightings:
 * where the robot is now and how fast it moves. Each sighting is off by its
 * bearing_error; the line averages that out. Returns the sightings it used. */
static int fit(cy_robot *robot,int32_t slot,int32_t tick,cy_vec3 *position,cy_vec3 *velocity) {
  cy_sighting all[32],*seen[32];uint32_t total=0;int32_t rate=0;int recent=0,n=0;double best=INFINITY,t0=0,spread=0;
  if(cy_tracker_history(robot,slot,all,32,&total)!=CY_OK||cy_match_tick_rate(robot,&rate)!=CY_OK)return 0;
  for(uint32_t i=0;i<total;i++)if(all[i].tick>tick-WINDOW){recent++;best=fmin(best,all[i].bearing_error);}
  if(recent<2)return 0;
  for(uint32_t i=0;i<total;i++)if(all[i].tick>tick-WINDOW&&all[i].bearing_error<=1.5*best){seen[n++]=&all[i];t0+=all[i].tick;}
  if(n<2)return 0;
  t0/=n;for(int i=0;i<n;i++)spread+=(seen[i]->tick-t0)*(seen[i]->tick-t0);
  double now[3],slope[3];
  for(int axis=0;axis<3;axis++) {
    double mean=0,along=0;
    for(int i=0;i<n;i++)mean+=(&seen[i]->position.x)[axis];mean/=n;
    for(int i=0;i<n;i++)along+=(seen[i]->tick-t0)*((&seen[i]->position.x)[axis]-mean);
    slope[axis]=along/spread;now[axis]=mean+slope[axis]*(tick-t0);
  }
  *position=(cy_vec3){now[0],now[1],now[2]};*velocity=(cy_vec3){slope[0]*rate,slope[1]*rate,slope[2]*rate};
  return n;
}
static int dodge_ticks=0,dodges=0,slot=0;
static void turn(cy_robot *robot) {
  cy_vec3 position={0},gravity={0};cy_contact target={0};cy_cue cue={0};cy_scan last={0};
  cy_me_position(robot,&position);cy_arena_gravity(robot,&gravity);cy_scanner_last(robot,&last);
  const double heading=get(robot,cy_me_heading),pitch=get(robot,cy_me_pitch);
  int32_t is_3d=0,tick=0;cy_match_is_3d(robot,&is_3d);cy_clock_tick(robot,&tick);
  int free_flight=is_3d&&gravity.z==0;cy_drive_brake(robot,0.2);
  int32_t last_seen=0;int lost=slot!=0&&cy_tracker_last_seen(robot,slot,&last_seen)==CY_OK&&tick-last_seen<=REACQUIRE;
  if(cy_sensors_nearest(robot,&target)==CY_OK) {
    slot=target.slot;
    double throttle=target.range>350?0.7:target.range<200?-0.4:0.2;
    drive(robot,throttle,steer(target.bearing,heading),steer(target.elevation,pitch));
    /* Halve the beam each tick until it is HOLD_WIDTH across at the target:
     * narrow enough for a precise bearing, wide enough to keep it in. */
    double hold=fmax(MIN_ARC,2*atan2(HOLD_WIDTH/2,target.range)*57.29577951308232);
    /* In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks, widen it
     * for a tick: a nearer robot it finds becomes the target. */
    double arc=tick%LOOK_EVERY==0?SEARCH_ARC:fmax(hold,fmin(SEARCH_ARC,last.arc/2));
    scan(robot,target.bearing,arc,target.elevation);
    double power=0;cy_gun_power_for_flight_time(robot,target.range,1,&power);
    /* An exact contact: gun.lead is the intercept if the target holds course.
     * A noisy one: lead the fitted line, and fire only once it rests on enough sightings. */
    cy_solution aim={0};cy_vec3 where,speed;int aimed,ready=1;
    if(target.bearing_error>0){int count=fit(robot,target.slot,tick,&where,&speed);aimed=count>0&&cy_gun_lead_point(robot,&where,&speed,power,&aim)==CY_OK;ready=aimed&&count>=SIGHTINGS;}
    else aimed=cy_gun_lead(robot,&target,power,&aim)==CY_OK;
    if(!aimed){aim.heading=target.bearing;aim.pitch=target.elevation;}
    cy_gun_aim_heading(robot,aim.heading);cy_gun_aim_pitch(robot,aim.pitch);
    cy_direction now={0};cy_gun_aim_now(robot,&now);double delta=0;cy_angles_angle_diff(aim.heading,now.heading,&delta);
    int32_t can_fire=0;cy_gun_can_fire(robot,power,10,&can_fire);
    if(ready&&fabs(delta)<3&&fabs(aim.pitch-now.pitch)<3&&can_fire){cy_gun_fire(robot);cy_gun_power(robot,power);}
    cy_status_text(robot,ready?"TRACK":"NARROW",ready?5:6);cy_status_value(robot,target.range);
  } else if(lost) {
    /* Lost it: look again where it was last seen, widening the beam, before searching. */
    cy_vec3 where={0};double bearing=0,elevation=0;cy_tracker_position(robot,slot,&where);cy_angles_bearing_to(&position,&where,&bearing);
    if(free_flight)cy_angles_elevation_to(&position,&where,&elevation);
    drive(robot,0.5,steer(bearing,heading),steer(elevation,pitch));scan(robot,bearing,fmin(SEARCH_ARC,last.arc*2),elevation);
    cy_status_text(robot,"REACQUIRE",9);
  } else if(cy_tracker_cue(robot,&cue)==CY_OK) {
    double bearing=0,distance=0,elevation=0,arc=0;
    cy_angles_bearing_to(&position,&cue.position,&bearing);cy_angles_distance(&position,&cue.position,&distance);
    if(free_flight)cy_angles_elevation_to(&position,&cue.position,&elevation);
    double error=fmin(cue.bearing_error,30),across=(tick%5-2)/2.0*error;
    double up=free_flight?((tick/5)%3-1)*error/2:0;
    drive(robot,0.7,steer(bearing,heading),steer(elevation,pitch));
    cy_scanner_arc_for_reach(robot,distance+100,&arc);scan(robot,bearing+across,arc,elevation+up);
    cy_status_text(robot,"CUE",3);cy_status_value(robot,distance);
  } else {
    double sweep=0;cy_angles_normalize(last.heading+36,&sweep);
    drive(robot,0.5,0.33,steer(0,pitch));scan(robot,sweep,SEARCH_ARC,free_flight?((tick/10)%3-1)*35:0);
    cy_status_text(robot,"SEARCH",6);
  }
  /* A simple dodge: vary it, or predict the shooter. */
  cy_contact cues[32];uint32_t count=0;int fired=0;
  if(cy_sensors_cues(robot,cues,32,&count)==CY_OK)for(uint32_t i=0;i<count;i++)fired|=cues[i].source==CY_SOURCE_MUZZLE;
  if(dodge_ticks==0&&fired){dodges++;dodge_ticks=10;}
  if(dodge_ticks>0) {
    /* Odd dodges go left, even ones right; turn +1 and strafe -1 go left. */
    int left=dodges%2==1;int32_t control=0;cy_match_control(robot,&control);dodge_ticks--;
    if(control==CY_CONTROL_FORCES){cy_drive_throttle(robot,0.5);cy_drive_strafe(robot,left?-1:1);}
    else{cy_drive_throttle(robot,1);cy_drive_turn(robot,left?1:-1);}
    cy_status_text(robot,left?"dodge:left":"dodge:right",left?10:11);cy_status_value(robot,dodges);
  }
  if(get(robot,cy_battery_charge)<get(robot,cy_battery_capacity)/4)cy_drive_throttle(robot,get(robot,cy_drive_get_throttle)*0.5);
  cy_storm storm={0};double margin=0;
  if(cy_nav_storm(robot,&storm)==CY_OK&&cy_nav_storm_margin(robot,&position,&margin)==CY_OK&&margin>-60) {
    double away=0,delta=0,control=0;cy_angles_bearing_to(&storm.center,&position,&away);cy_angles_angle_diff(away,heading,&delta);cy_angles_clamp(delta/30,-1,1,&control);
    cy_drive_throttle(robot,0.8);cy_drive_turn(robot,control);
  }
  cy_zone zone={0};
  if(cy_nav_zone(robot,&zone)==CY_OK&&cy_nav_zone_margin(robot,&position,&margin)==CY_OK&&margin<60) {
    double bearing=0,climb=0,delta=0,turn_control=0,pitch_control=0;
    cy_angles_bearing_to(&position,&zone.center,&bearing);if(zone.shape==CY_SHAPE_SPHERE)cy_angles_elevation_to(&position,&zone.center,&climb);
    cy_angles_angle_diff(bearing,heading,&delta);cy_angles_clamp(delta/30,-1,1,&turn_control);cy_angles_clamp((climb-pitch)/30,-1,1,&pitch_control);
    drive(robot,0.8,turn_control,pitch_control);
  }
}
int main(void) {
  cy_robot *robot=0;if(cy_open(CY_API_MAJOR,CY_API_MINOR,"clunkyard-c",11,CY_API_VERSION,sizeof CY_API_VERSION-1,&robot)<0)return 1;
  int32_t event;
  for(;;){int32_t status=cy_session_next(robot,&event);if(status==CY_E_INTERNAL){if(cy_session_send(robot)<0)break;continue;}if(status!=CY_OK||event==CY_EVENT_END)break;if(event==CY_EVENT_STATE)turn(robot);if(cy_session_send(robot)<0)break;}
  cy_close(robot);return 0;
}
