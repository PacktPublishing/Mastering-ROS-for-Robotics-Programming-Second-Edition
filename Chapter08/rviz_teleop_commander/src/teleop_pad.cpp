/*
 * Copyright (C) 2017, Jonathan Cacace

 * Email id : jonathan.cacace@gmail.com

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

*/


#include "teleop_pad.h"

namespace rviz_teleop_commander
{

	TeleopPanel::TeleopPanel( QWidget* parent )
		: rviz::Panel( parent )
		, linear_velocity_( 0 )
		, angular_velocity_( 0 )
	{

		QVBoxLayout* topic_layout = new QVBoxLayout;
		topic_layout->addWidget( new QLabel( "Teleop Topic:" ));
		output_topic_editor_ = new QLineEdit;
		topic_layout->addWidget( output_topic_editor_ );


		topic_layout->addWidget( new QLabel( "Linear Velocity:" ));
		output_topic_editor_1 = new QLineEdit;
		topic_layout->addWidget( output_topic_editor_1 );


		topic_layout->addWidget( new QLabel( "Angular Velocity:" ));
		output_topic_editor_2 = new QLineEdit;
		topic_layout->addWidget( output_topic_editor_2 );

		QHBoxLayout* layout = new QHBoxLayout;
		layout->addLayout( topic_layout );
		setLayout( layout );


		QTimer* output_timer = new QTimer( this );

		// Next we make signal/slot connections.
		connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
		connect( output_topic_editor_1, SIGNAL( editingFinished() ), this, SLOT( update_Linear_Velocity() ));
		connect( output_topic_editor_2, SIGNAL( editingFinished() ), this, SLOT( update_Angular_Velocity() ));

		connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
		// Start the timer.
		output_timer->start( 100 );
	}

	void TeleopPanel::update_Linear_Velocity()
	{
		  QString temp_string = output_topic_editor_1->text();	
		  float lin = temp_string.toFloat();  
		  linear_velocity_ = lin;
	}

	void TeleopPanel::update_Angular_Velocity()
	{
		  QString temp_string = output_topic_editor_2->text();
		  float ang = temp_string.toFloat() ;  
		  angular_velocity_ = ang;
	}

	void TeleopPanel::updateTopic()
	{
		setTopic( output_topic_editor_->text() );
	}

	// Set the topic name we are publishing to.
	void TeleopPanel::setTopic( const QString& new_topic )
	{
		// Only take action if the name has changed.
		if( new_topic != output_topic_ )
		{
		  output_topic_ = new_topic;
		  // If the topic is the empty string, don't publish anything.
		  if( output_topic_ == "" )
		  {
		    velocity_publisher_.shutdown();
		  }
		  else
		  {

		    velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
		  }

		  Q_EMIT configChanged();
		}

		// Gray out the control widget when the output topic is empty.
	}

	void TeleopPanel::sendVel()
	{
		if( ros::ok() && velocity_publisher_ )
		{
		  geometry_msgs::Twist msg;
		  msg.linear.x = linear_velocity_;
		  msg.linear.y = 0;
		  msg.linear.z = 0;
		  msg.angular.x = 0;
		  msg.angular.y = 0;
		  msg.angular.z = angular_velocity_;
		  velocity_publisher_.publish( msg );
		}
	}

	void TeleopPanel::save( rviz::Config config ) const
	{
		rviz::Panel::save( config );
		config.mapSetValue( "Topic", output_topic_ );
	}

	// Load all configuration data for this panel from the given Config object.
	void TeleopPanel::load( const rviz::Config& config )
	{
		rviz::Panel::load( config );
		QString topic;
		if( config.mapGetString( "Topic", &topic ))
		{
		  output_topic_editor_->setText( topic );
		  updateTopic();
		}
	}

} // end namespace rviz_plugin_tutorials


PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel,rviz::Panel )


