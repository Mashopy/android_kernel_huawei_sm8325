

#ifndef _HME_VQM_REPORT_H_
#define _HME_VQM_REPORT_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#define HME_VQM_EXCEPTION_TIME_BUF_SIZE 8

#define HME_VQM_EXCEPTION_DETAILS_NUM 42

// occupy 2 digits in decimal
enum bsd_phenomenon {
	BSD_PHENOMENON_INVALID = 0,
	BSD_PHENOMENON_NOSOUND = 1,
	BSD_PHENOMENON_INTERRUPT = 2,
	BSD_PHENOMENON_NOISE = 3,
	BSD_PHENOMENON_ELECTRIC_NOISE = 4,
	BSD_PHENOMENON_BLUR = 5,
	BSD_PHENOMENON_WRR_NOISE = 6,
	BSD_PHENOMENON_VQI_LOW_SCORE = 8,
	BSD_PHENOMENON_TIMING_CONFLICTS = 9,
	BSD_PHENOMENON_WHISTLE = 10,
	BSD_PHENOMENON_ECHO = 11,
};

enum bsd_position {
	BSD_POS_INVALID = 0,
	BSD_POS_MICIN = 1,
	BSD_POS_SPKOUT = 2,
	BSD_POS_CODEIN = 3,
	BSD_POS_CODEOUT = 4,
	BSD_POS_ECREF = 5,
	BSD_POS_LINEIN = 6,
	BSD_POS_LINEOUT = 7,
	BSD_POS_MODEM = 8,
	BSD_POS_HIFI = 9,
	BSD_POS_DMA = 10,
	BSD_POS_CODEC = 11,
};

typedef struct _hme_vqm_signal_status_stru {
	unsigned int total_frame_cnt;
	unsigned int exp_status_flags;

	unsigned short exp_params_of_details[HME_VQM_EXCEPTION_DETAILS_NUM];

	unsigned int exception_start_time_buf[HME_VQM_EXCEPTION_TIME_BUF_SIZE];
	unsigned int exception_end_time_buf[HME_VQM_EXCEPTION_TIME_BUF_SIZE];
	unsigned int exception_buf_idx;

	unsigned char inactive_voice_section;
	unsigned char exp_params_index;
	unsigned char event_cause;
	unsigned char reserved;
} hme_vqm_signal_status;

typedef struct _hme_vqm_network_state_stru {
	unsigned int burst_count;
	unsigned int burst_duration;
	unsigned int gap_duration;
	unsigned char burst_density;
	unsigned char gap_density;
	unsigned char lost_rate;
	unsigned char discard_rate;
} hme_vqm_network_state;

typedef struct _hme_vqm_volte_info {
	unsigned int rcv_packets;
	unsigned int lost_packets;
	unsigned int jb_lost_packets;
	unsigned int plc_cnt;
	unsigned int tsm_cnt;
} hme_vqm_volte_info;

typedef struct _hme_vqm_result_stru {
	int16_t i_version;
	unsigned short result;
	int i_diagnoses_pos;
	int i_session_is_over;

	hme_vqm_signal_status up_status;
	hme_vqm_signal_status dn_status;

	hme_vqm_network_state net_work_improved_state;
	hme_vqm_network_state net_work_original_state;

	hme_vqm_volte_info volte_info;
	unsigned short alert_level;
	unsigned short reserved;
} hme_vqm_result;

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif
