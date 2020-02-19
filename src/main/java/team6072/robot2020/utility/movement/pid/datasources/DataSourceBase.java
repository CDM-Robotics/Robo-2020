package team6072.robot2020.utility.movement.pid.datasources;

/**
 * This is an abstract class that allows other systems extract data in cleaner manner than directly calling it.
 */
public abstract class DataSourceBase {

    public abstract double getData();

}