/* Generated by itemis CREATE code generator. */


#include "../src/sc_types.h"

#include "Statechart.h"
#include "Statechart_required.h"

/*! \file
Implementation of the state machine 'Statechart'
*/

#ifndef SC_UNUSED
#define SC_UNUSED(P) (void)(P)
#endif

/* prototypes of all internal functions */
static void enact_main_region_Inicializacija(Statechart* handle);
static void enact_main_region_Read_Accel(Statechart* handle);
static void enact_main_region_Calculate_Grav(Statechart* handle);
static void enact_main_region_Display_Grav(Statechart* handle);
static void enact_main_region_Disp_LCD(Statechart* handle);
static void enact_main_region_Uart(Statechart* handle);
static void enact_main_region_Siuntimas(Statechart* handle);
static void enact_main_region_Kalibravimas(Statechart* handle);
static void exact_main_region_Disp_LCD(Statechart* handle);
static void enseq_main_region_Inicializacija_default(Statechart* handle);
static void enseq_main_region_Read_Accel_default(Statechart* handle);
static void enseq_main_region_Disp_LCD_default(Statechart* handle);
static void enseq_main_region_Uart_default(Statechart* handle);
static void enseq_main_region_Siuntimas_default(Statechart* handle);
static void enseq_main_region_default(Statechart* handle);
static void exseq_main_region_Inicializacija(Statechart* handle);
static void exseq_main_region_Read_Accel(Statechart* handle);
static void exseq_main_region_Calculate_Grav(Statechart* handle);
static void exseq_main_region_Display_Grav(Statechart* handle);
static void exseq_main_region_Disp_LCD(Statechart* handle);
static void exseq_main_region_Uart(Statechart* handle);
static void exseq_main_region_Siuntimas(Statechart* handle);
static void exseq_main_region_Kalibravimas(Statechart* handle);
static void exseq_main_region(Statechart* handle);
static void react_main_region__entry_Default(Statechart* handle);

/*! The reactions of state Inicializacija. */
static sc_integer main_region_Inicializacija_react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Read_Accel. */
static sc_integer main_region_Read_Accel_react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Calculate_Grav. */
static sc_integer main_region_Calculate_Grav_react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Display_Grav. */
static sc_integer main_region_Display_Grav_react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Disp_LCD. */
static sc_integer main_region_Disp_LCD_react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Uart. */
static sc_integer main_region_Uart_react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Siuntimas. */
static sc_integer main_region_Siuntimas_react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Kalibravimas. */
static sc_integer main_region_Kalibravimas_react(Statechart* handle, const sc_integer transitioned_before);


static void clear_in_events(Statechart* handle);

static void micro_step(Statechart* handle);

/*! Performs a 'run to completion' step. */
static void run_cycle(Statechart* handle);




static void statechart_internal_set_sample_no(Statechart* handle, sc_integer value)
;


static void statechart_eventqueue_init(statechart_eventqueue * eq, statechart_event *buffer, sc_integer capacity);
static sc_integer statechart_eventqueue_size(statechart_eventqueue * eq);
static void statechart_event_init(statechart_event * ev, StatechartEventID name);
static statechart_event statechart_eventqueue_pop(statechart_eventqueue * eq);
static sc_boolean statechart_eventqueue_push(statechart_eventqueue * eq, statechart_event ev);
static void statechart_add_event_to_queue(statechart_eventqueue * eq, StatechartEventID name);
static sc_boolean statechart_dispatch_event(Statechart* handle, const statechart_event * event);
static statechart_event statechart_get_next_event(Statechart* handle);
static sc_boolean statechart_dispatch_next_event(Statechart* handle);


void statechart_init(Statechart* handle)
{
	sc_integer i;
	
	for (i = 0; i < STATECHART_MAX_ORTHOGONAL_STATES; ++i)
	{
		handle->stateConfVector[i] = Statechart_last_state;
	}
	
				
	clear_in_events(handle);
	
	
	/* Default init sequence for statechart Statechart */
	statechart_internal_set_sample_no(handle, 0);
	
	handle->isExecuting = bool_false;
	statechart_eventqueue_init(&handle->in_event_queue, handle->in_buffer, STATECHART_IN_EVENTQUEUE_BUFFERSIZE);
}

void statechart_enter(Statechart* handle)
{
	/* Activates the state machine. */
	if (handle->isExecuting == bool_true)
	{ 
		return;
	} 
	handle->isExecuting = bool_true;
	/* Default enter sequence for statechart Statechart */
	enseq_main_region_default(handle);
	handle->doCompletion = bool_false;
	do
	{ 
		if (handle->completed == bool_true)
		{ 
			handle->doCompletion = bool_true;
		} 
		handle->completed = bool_false;
		micro_step(handle);
		clear_in_events(handle);
		handle->doCompletion = bool_false;
	} while (handle->completed == bool_true);
	handle->isExecuting = bool_false;
}

void statechart_exit(Statechart* handle)
{
	/* Deactivates the state machine. */
	if (handle->isExecuting == bool_true)
	{ 
		return;
	} 
	handle->isExecuting = bool_true;
	/* Default exit sequence for statechart Statechart */
	exseq_main_region(handle);
	handle->stateConfVector[0] = Statechart_last_state;
	handle->isExecuting = bool_false;
}

/*!
Can be used by the client code to trigger a run to completion step without raising an event.
*/
void statechart_trigger_without_event(Statechart* handle) {
	run_cycle(handle);
}


sc_boolean statechart_is_active(const Statechart* handle)
{
	sc_boolean result = bool_false;
	sc_integer i;
	
	for(i = 0; i < STATECHART_MAX_ORTHOGONAL_STATES; i++)
	{
		result = result || handle->stateConfVector[i] != Statechart_last_state;
	}
	
	return result;
}

/* 
 * Always returns 'false' since this state machine can never become final.
 */
sc_boolean statechart_is_final(const Statechart* handle)
{
	SC_UNUSED(handle);
	return bool_false;
}

sc_boolean statechart_is_state_active(const Statechart* handle, StatechartStates state)
{
	sc_boolean result = bool_false;
	switch (state)
	{
		case Statechart_main_region_Inicializacija :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_MAIN_REGION_INICIALIZACIJA] == Statechart_main_region_Inicializacija
			);
				break;
		case Statechart_main_region_Read_Accel :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_MAIN_REGION_READ_ACCEL] == Statechart_main_region_Read_Accel
			);
				break;
		case Statechart_main_region_Calculate_Grav :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_MAIN_REGION_CALCULATE_GRAV] == Statechart_main_region_Calculate_Grav
			);
				break;
		case Statechart_main_region_Display_Grav :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_MAIN_REGION_DISPLAY_GRAV] == Statechart_main_region_Display_Grav
			);
				break;
		case Statechart_main_region_Disp_LCD :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_MAIN_REGION_DISP_LCD] == Statechart_main_region_Disp_LCD
			);
				break;
		case Statechart_main_region_Uart :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_MAIN_REGION_UART] == Statechart_main_region_Uart
			);
				break;
		case Statechart_main_region_Siuntimas :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_MAIN_REGION_SIUNTIMAS] == Statechart_main_region_Siuntimas
			);
				break;
		case Statechart_main_region_Kalibravimas :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_MAIN_REGION_KALIBRAVIMAS] == Statechart_main_region_Kalibravimas
			);
				break;
			default:
				result = bool_false;
				break;
		}
		return result;
	}

static void clear_in_events(Statechart* handle)
{
	handle->iface.Timer_raised = bool_false;
	handle->iface.Send_raised = bool_false;
}

static void micro_step(Statechart* handle)
{
	switch(handle->stateConfVector[ 0 ])
	{
		case Statechart_main_region_Inicializacija :
		{
			main_region_Inicializacija_react(handle,-1);
			break;
		}
		case Statechart_main_region_Read_Accel :
		{
			main_region_Read_Accel_react(handle,-1);
			break;
		}
		case Statechart_main_region_Calculate_Grav :
		{
			main_region_Calculate_Grav_react(handle,-1);
			break;
		}
		case Statechart_main_region_Display_Grav :
		{
			main_region_Display_Grav_react(handle,-1);
			break;
		}
		case Statechart_main_region_Disp_LCD :
		{
			main_region_Disp_LCD_react(handle,-1);
			break;
		}
		case Statechart_main_region_Uart :
		{
			main_region_Uart_react(handle,-1);
			break;
		}
		case Statechart_main_region_Siuntimas :
		{
			main_region_Siuntimas_react(handle,-1);
			break;
		}
		case Statechart_main_region_Kalibravimas :
		{
			main_region_Kalibravimas_react(handle,-1);
			break;
		}
		default: 
			/* do nothing */
			break;
	}
}

static void run_cycle(Statechart* handle)
{
	/* Performs a 'run to completion' step. */
	if (handle->isExecuting == bool_true)
	{ 
		return;
	} 
	handle->isExecuting = bool_true;
	statechart_dispatch_next_event(handle);
	do
	{ 
		handle->doCompletion = bool_false;
		do
		{ 
			if (handle->completed == bool_true)
			{ 
				handle->doCompletion = bool_true;
			} 
			handle->completed = bool_false;
			micro_step(handle);
			clear_in_events(handle);
			handle->doCompletion = bool_false;
		} while (handle->completed == bool_true);
	} while (statechart_dispatch_next_event(handle) == bool_true);
	handle->isExecuting = bool_false;
}


void statechart_raise_timer(Statechart* handle)
{
	statechart_add_event_to_queue(&(handle->in_event_queue), Statechart_Timer);
	run_cycle(handle);
}

void statechart_raise_send(Statechart* handle)
{
	statechart_add_event_to_queue(&(handle->in_event_queue), Statechart_Send);
	run_cycle(handle);
}




static void statechart_internal_set_sample_no(Statechart* handle, sc_integer value)
{
	handle->internal.sample_no = value;
}


/* implementations of all internal functions */

static void enact_main_region_Inicializacija(Statechart* handle)
{
	/* Entry action for state 'Inicializacija'. */
	statechart_mPU6050_Init(handle);
	statechart_lCD_Init(handle);
	handle->completed = bool_true;
}

static void enact_main_region_Read_Accel(Statechart* handle)
{
	/* Entry action for state 'Read_Accel'. */
	statechart_mPU6050_Read_Accel(handle);
	handle->completed = bool_true;
}

static void enact_main_region_Calculate_Grav(Statechart* handle)
{
	/* Entry action for state 'Calculate_Grav'. */
	statechart_calculate_Gravity(handle);
	handle->completed = bool_true;
}

/* Entry action for state 'Display_Grav'. */
static void enact_main_region_Display_Grav(Statechart* handle)
{
	/* Entry action for state 'Display_Grav'. */
	statechart_display_Grvt(handle);
	statechart_rMS(handle);
}

static void enact_main_region_Disp_LCD(Statechart* handle)
{
	/* Entry action for state 'Disp_LCD'. */
	statechart_display_LCD(handle);
	handle->completed = bool_true;
}

static void enact_main_region_Uart(Statechart* handle)
{
	/* Entry action for state 'Uart'. */
	statechart_send_Uart(handle);
	handle->completed = bool_true;
}

static void enact_main_region_Siuntimas(Statechart* handle)
{
	/* Entry action for state 'Siuntimas'. */
	statechart_set_accel_range(handle);
	handle->completed = bool_true;
}

static void enact_main_region_Kalibravimas(Statechart* handle)
{
	/* Entry action for state 'Kalibravimas'. */
	statechart_mPU6050_Calibrate(handle);
	handle->completed = bool_true;
}

/* Exit action for state 'Disp_LCD'. */
static void exact_main_region_Disp_LCD(Statechart* handle)
{
	/* Exit action for state 'Disp_LCD'. */
	statechart_internal_set_sample_no(handle, 0);
}

/* 'default' enter sequence for state Inicializacija */
static void enseq_main_region_Inicializacija_default(Statechart* handle)
{
	/* 'default' enter sequence for state Inicializacija */
	enact_main_region_Inicializacija(handle);
	handle->stateConfVector[0] = Statechart_main_region_Inicializacija;
}

/* 'default' enter sequence for state Read_Accel */
static void enseq_main_region_Read_Accel_default(Statechart* handle)
{
	/* 'default' enter sequence for state Read_Accel */
	enact_main_region_Read_Accel(handle);
	handle->stateConfVector[0] = Statechart_main_region_Read_Accel;
}

/* 'default' enter sequence for state Disp_LCD */
static void enseq_main_region_Disp_LCD_default(Statechart* handle)
{
	/* 'default' enter sequence for state Disp_LCD */
	enact_main_region_Disp_LCD(handle);
	handle->stateConfVector[0] = Statechart_main_region_Disp_LCD;
}

/* 'default' enter sequence for state Uart */
static void enseq_main_region_Uart_default(Statechart* handle)
{
	/* 'default' enter sequence for state Uart */
	enact_main_region_Uart(handle);
	handle->stateConfVector[0] = Statechart_main_region_Uart;
}

/* 'default' enter sequence for state Siuntimas */
static void enseq_main_region_Siuntimas_default(Statechart* handle)
{
	/* 'default' enter sequence for state Siuntimas */
	enact_main_region_Siuntimas(handle);
	handle->stateConfVector[0] = Statechart_main_region_Siuntimas;
}

/* 'default' enter sequence for region main region */
static void enseq_main_region_default(Statechart* handle)
{
	/* 'default' enter sequence for region main region */
	react_main_region__entry_Default(handle);
}

/* Default exit sequence for state Inicializacija */
static void exseq_main_region_Inicializacija(Statechart* handle)
{
	/* Default exit sequence for state Inicializacija */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state Read_Accel */
static void exseq_main_region_Read_Accel(Statechart* handle)
{
	/* Default exit sequence for state Read_Accel */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state Calculate_Grav */
static void exseq_main_region_Calculate_Grav(Statechart* handle)
{
	/* Default exit sequence for state Calculate_Grav */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state Display_Grav */
static void exseq_main_region_Display_Grav(Statechart* handle)
{
	/* Default exit sequence for state Display_Grav */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state Disp_LCD */
static void exseq_main_region_Disp_LCD(Statechart* handle)
{
	/* Default exit sequence for state Disp_LCD */
	handle->stateConfVector[0] = Statechart_last_state;
	exact_main_region_Disp_LCD(handle);
}

/* Default exit sequence for state Uart */
static void exseq_main_region_Uart(Statechart* handle)
{
	/* Default exit sequence for state Uart */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state Siuntimas */
static void exseq_main_region_Siuntimas(Statechart* handle)
{
	/* Default exit sequence for state Siuntimas */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state Kalibravimas */
static void exseq_main_region_Kalibravimas(Statechart* handle)
{
	/* Default exit sequence for state Kalibravimas */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for region main region */
static void exseq_main_region(Statechart* handle)
{
	/* Default exit sequence for region main region */
	/* Handle exit of all possible states (of Statechart.main_region) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Statechart_main_region_Inicializacija :
		{
			exseq_main_region_Inicializacija(handle);
			break;
		}
		case Statechart_main_region_Read_Accel :
		{
			exseq_main_region_Read_Accel(handle);
			break;
		}
		case Statechart_main_region_Calculate_Grav :
		{
			exseq_main_region_Calculate_Grav(handle);
			break;
		}
		case Statechart_main_region_Display_Grav :
		{
			exseq_main_region_Display_Grav(handle);
			break;
		}
		case Statechart_main_region_Disp_LCD :
		{
			exseq_main_region_Disp_LCD(handle);
			break;
		}
		case Statechart_main_region_Uart :
		{
			exseq_main_region_Uart(handle);
			break;
		}
		case Statechart_main_region_Siuntimas :
		{
			exseq_main_region_Siuntimas(handle);
			break;
		}
		case Statechart_main_region_Kalibravimas :
		{
			exseq_main_region_Kalibravimas(handle);
			break;
		}
		default: 
			/* do nothing */
			break;
	}
}

/* Default react sequence for initial entry  */
static void react_main_region__entry_Default(Statechart* handle)
{
	/* Default react sequence for initial entry  */
	enseq_main_region_Inicializacija_default(handle);
}


static sc_integer main_region_Inicializacija_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Inicializacija. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state Inicializacija */
		handle->stateConfVector[0] = Statechart_last_state;
		/* 'default' enter sequence for state Kalibravimas */
		enact_main_region_Kalibravimas(handle);
		handle->stateConfVector[0] = Statechart_main_region_Kalibravimas;
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = transitioned_before;
	}
	return transitioned_after;
}

static sc_integer main_region_Read_Accel_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Read_Accel. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state Read_Accel */
		handle->stateConfVector[0] = Statechart_last_state;
		/* 'default' enter sequence for state Calculate_Grav */
		enact_main_region_Calculate_Grav(handle);
		handle->stateConfVector[0] = Statechart_main_region_Calculate_Grav;
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = transitioned_before;
	}
	return transitioned_after;
}

static sc_integer main_region_Calculate_Grav_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Calculate_Grav. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state Calculate_Grav */
		handle->stateConfVector[0] = Statechart_last_state;
		/* 'default' enter sequence for state Display_Grav */
		enact_main_region_Display_Grav(handle);
		handle->stateConfVector[0] = Statechart_main_region_Display_Grav;
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = transitioned_before;
	}
	return transitioned_after;
}

static sc_integer main_region_Display_Grav_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Display_Grav. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_false)
	{ 
		if ((transitioned_after) < (0))
		{ 
			if (handle->iface.Timer_raised == bool_true)
			{ 
				exseq_main_region_Display_Grav(handle);
				enseq_main_region_Uart_default(handle);
				transitioned_after = 0;
			} 
		} 
		/* If no transition was taken */
		if ((transitioned_after) == (transitioned_before))
		{ 
			/* then execute local reactions. */
			transitioned_after = transitioned_before;
		} 
	} return transitioned_after;
}

static sc_integer main_region_Disp_LCD_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Disp_LCD. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state Disp_LCD */
		handle->stateConfVector[0] = Statechart_last_state;
		exact_main_region_Disp_LCD(handle);
		/* 'default' enter sequence for state Read_Accel */
		enact_main_region_Read_Accel(handle);
		handle->stateConfVector[0] = Statechart_main_region_Read_Accel;
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = transitioned_before;
	}
	return transitioned_after;
}

static sc_integer main_region_Uart_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Uart. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state Uart */
		handle->stateConfVector[0] = Statechart_last_state;
		/* The reactions of state null. */
		if ((handle->internal.sample_no) == (30))
		{ 
			enseq_main_region_Disp_LCD_default(handle);
		}  else
		{
			if (handle->iface.Send_raised == bool_true)
			{ 
				enseq_main_region_Siuntimas_default(handle);
			}  else
			{
				statechart_internal_set_sample_no(handle, (handle->internal.sample_no + 1));
				enseq_main_region_Read_Accel_default(handle);
			}
		}
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = transitioned_before;
	}
	return transitioned_after;
}

static sc_integer main_region_Siuntimas_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Siuntimas. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state Siuntimas */
		handle->stateConfVector[0] = Statechart_last_state;
		/* 'default' enter sequence for state Kalibravimas */
		enact_main_region_Kalibravimas(handle);
		handle->stateConfVector[0] = Statechart_main_region_Kalibravimas;
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = transitioned_before;
	}
	return transitioned_after;
}

static sc_integer main_region_Kalibravimas_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Kalibravimas. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state Kalibravimas */
		handle->stateConfVector[0] = Statechart_last_state;
		/* 'default' enter sequence for state Read_Accel */
		enact_main_region_Read_Accel(handle);
		handle->stateConfVector[0] = Statechart_main_region_Read_Accel;
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = transitioned_before;
	}
	return transitioned_after;
}




static void statechart_eventqueue_init(statechart_eventqueue * eq, statechart_event *buffer, sc_integer capacity)
{
	eq->events = buffer;
	eq->capacity = capacity;
	eq->push_index = 0;
	eq->pop_index = 0;
	eq->size = 0;
}

static sc_integer statechart_eventqueue_size(statechart_eventqueue * eq)
{
	return eq->size;
}

static statechart_event statechart_eventqueue_pop(statechart_eventqueue * eq)
{
	statechart_event event;
	if(statechart_eventqueue_size(eq) <= 0) {
		statechart_event_init(&event, Statechart_invalid_event);
	}
	else {
		event = eq->events[eq->pop_index];
		
		if(eq->pop_index < eq->capacity - 1) {
			eq->pop_index++;
		} 
		else {
			eq->pop_index = 0;
		}
		eq->size--;
	}
	return event;
}
static sc_boolean statechart_eventqueue_push(statechart_eventqueue * eq, statechart_event ev)
{
	if(statechart_eventqueue_size(eq) == eq->capacity) {
		return bool_false;
	}
	else {
		eq->events[eq->push_index] = ev;
		
		if(eq->push_index < eq->capacity - 1) {
			eq->push_index++;
		}
		else {
			eq->push_index = 0;
		}
		eq->size++;
		
		return bool_true;
	}
}
static void statechart_event_init(statechart_event * ev, StatechartEventID name)
{
	ev->name = name;
}

static void statechart_add_event_to_queue(statechart_eventqueue * eq, StatechartEventID name)
{
	statechart_event event;
	statechart_event_init(&event, name);
	statechart_eventqueue_push(eq, event);
}

static sc_boolean statechart_dispatch_event(Statechart* handle, const statechart_event * event) {
	switch(event->name) {
		case Statechart_Timer:
		{
			handle->iface.Timer_raised = bool_true;
			return bool_true;
		}
		case Statechart_Send:
		{
			handle->iface.Send_raised = bool_true;
			return bool_true;
		}
		default:
			return bool_false;
	}
}

static statechart_event statechart_get_next_event(Statechart* handle)
{
	statechart_event next_event;
	statechart_event_init(&next_event, Statechart_invalid_event);
	if(statechart_eventqueue_size(&(handle->in_event_queue)) > 0) {
		next_event = statechart_eventqueue_pop(&(handle->in_event_queue));
	}
	return next_event;
}

static sc_boolean statechart_dispatch_next_event(Statechart* handle)
{
	statechart_event nextEvent;
	nextEvent = statechart_get_next_event(handle);
	return statechart_dispatch_event(handle, &nextEvent);
}
