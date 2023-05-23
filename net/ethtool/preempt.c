// SPDX-License-Identifier: GPL-2.0-only

#include "netlink.h"
#include "common.h"

struct preempt_req_info {
	struct ethnl_req_info		base;
};

struct preempt_reply_data {
	struct ethnl_reply_data		base;
	struct ethtool_fp		fp;
};

#define PREEMPT_REPDATA(__reply_base) \
	container_of(__reply_base, struct preempt_reply_data, base)

const struct nla_policy preempt_get_policy[ETHTOOL_A_PREEMPT_MAX + 1] = {
	[ETHTOOL_A_PREEMPT_UNSPEC]		= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_HEADER]		= { .type = NLA_NESTED },
	[ETHTOOL_A_PREEMPT_SUPPORTED]		= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_STATUS]		= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_ACTIVE]		= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_MIN_FRAG_SIZE]	= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_QUEUES_SUPPORTED]	= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_QUEUES_PREEMPTIBLE]	= { .type = NLA_REJECT },
};

static int preempt_prepare_data(const struct ethnl_req_info *req_base,
				struct ethnl_reply_data *reply_base,
				const struct genl_info *info)
{
	struct preempt_reply_data *data = PREEMPT_REPDATA(reply_base);
	struct net_device *dev = reply_base->dev;
	int ret;

	if (!dev->ethtool_ops->get_preempt)
		return -EOPNOTSUPP;

	ret = ethnl_ops_begin(dev);
	if (ret < 0)
		return ret;

	ret = dev->ethtool_ops->get_preempt(dev, &data->fp);
	ethnl_ops_complete(dev);

	return ret;
}

static int preempt_reply_size(const struct ethnl_req_info *req_base,
			      const struct ethnl_reply_data *reply_base)
{
	int len = 0;

	len += nla_total_size(sizeof(u8)); /* _PREEMPT_SUPPORTED */
	len += nla_total_size(sizeof(u8)); /* _PREEMPT_STATUS */
	len += nla_total_size(sizeof(u8)); /* _PREEMPT_ACTIVE */
	len += nla_total_size(sizeof(u32)); /* _PREEMPT_QUEUES_SUPPORTED */
	len += nla_total_size(sizeof(u32)); /* _PREEMPT_QUEUES_PREEMPTIBLE */
	len += nla_total_size(sizeof(u32)); /* _PREEMPT_MIN_FRAG_SIZE */

	return len;
}

static int preempt_fill_reply(struct sk_buff *skb,
			      const struct ethnl_req_info *req_base,
			      const struct ethnl_reply_data *reply_base)
{
	const struct preempt_reply_data *data = PREEMPT_REPDATA(reply_base);
	const struct ethtool_fp *preempt = &data->fp;

	if (nla_put_u32(skb, ETHTOOL_A_PREEMPT_QUEUES_SUPPORTED,
			  preempt->supported_queues_mask))
		return -EMSGSIZE;

	if (nla_put_u32(skb, ETHTOOL_A_PREEMPT_QUEUES_PREEMPTIBLE,
			  preempt->preemptible_queues_mask))
		return -EMSGSIZE;

	if (nla_put_u8(skb, ETHTOOL_A_PREEMPT_STATUS, preempt->fp_status))
		return -EMSGSIZE;

	if (nla_put_u8(skb, ETHTOOL_A_PREEMPT_ACTIVE, preempt->fp_active))
		return -EMSGSIZE;

	if (nla_put_u8(skb, ETHTOOL_A_PREEMPT_SUPPORTED,
		       preempt->fp_supported))
		return -EMSGSIZE;

	if (nla_put_u32(skb, ETHTOOL_A_PREEMPT_MIN_FRAG_SIZE,
			preempt->min_frag_size))
		return -EMSGSIZE;

	return 0;
}

const struct ethnl_request_ops ethnl_preempt_request_ops = {
	.request_cmd		= ETHTOOL_MSG_PREEMPT_GET,
	.reply_cmd		= ETHTOOL_MSG_PREEMPT_GET_REPLY,
	.hdr_attr		= ETHTOOL_A_PREEMPT_HEADER,
	.req_info_size		= sizeof(struct preempt_req_info),
	.reply_data_size	= sizeof(struct preempt_reply_data),

	.prepare_data		= preempt_prepare_data,
	.reply_size		= preempt_reply_size,
	.fill_reply		= preempt_fill_reply,
};

static const struct nla_policy
preempt_set_policy[ETHTOOL_A_PREEMPT_MAX + 1] = {
	[ETHTOOL_A_PREEMPT_UNSPEC]			= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_HEADER]			= { .type = NLA_NESTED },
	[ETHTOOL_A_PREEMPT_DISABLED]			= { .type = NLA_FLAG },
	[ETHTOOL_A_PREEMPT_SUPPORTED]			= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_STATUS]			= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_ACTIVE]			= { .type = NLA_U8 },
	[ETHTOOL_A_PREEMPT_MIN_FRAG_SIZE]		= { .type = NLA_U32 },
	[ETHTOOL_A_PREEMPT_QUEUES_SUPPORTED]		= { .type = NLA_REJECT },
	[ETHTOOL_A_PREEMPT_QUEUES_PREEMPTIBLE]		= { .type = NLA_U32 },
};

int ethnl_set_preempt(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *tb[ARRAY_SIZE(preempt_set_policy)];
	struct ethtool_fp preempt = {};
	struct ethnl_req_info req_info = {};
	struct net_device *dev;
	bool mod = false;
	int ret;

	ret = nlmsg_parse(info->nlhdr, GENL_HDRLEN, tb,
			  ETHTOOL_A_PREEMPT_MAX, preempt_set_policy,
			  info->extack);
	if (ret < 0)
		return ret;

	ret = ethnl_parse_header_dev_get(&req_info,
					 tb[ETHTOOL_A_PREEMPT_HEADER],
					 genl_info_net(info), info->extack,
					 true);
	if (ret < 0)
		return ret;
	dev = req_info.dev;
	ret = -EOPNOTSUPP;
	if (!dev->ethtool_ops->get_preempt ||
	    !dev->ethtool_ops->set_preempt)
		goto out_dev;

	rtnl_lock();
	ret = ethnl_ops_begin(dev);
	if (ret < 0)
		goto out_rtnl;

	if (nla_get_flag(tb[ETHTOOL_A_PREEMPT_DISABLED])) {
		preempt.disabled = 1;
		mod = true;
	}
	if (tb[ETHTOOL_A_PREEMPT_ACTIVE]) {
		ethnl_update_u8(&preempt.fp_enabled,
				tb[ETHTOOL_A_PREEMPT_ACTIVE], &mod);
		mod = true;
	}
	if (tb[ETHTOOL_A_PREEMPT_MIN_FRAG_SIZE])
		ethnl_update_u32(&preempt.min_frag_size,
				 tb[ETHTOOL_A_PREEMPT_MIN_FRAG_SIZE], &mod);
	else
		preempt.min_frag_size = 60;
	if (tb[ETHTOOL_A_PREEMPT_QUEUES_PREEMPTIBLE]) {
		ethnl_update_u32(&preempt.preemptible_queues_mask,
				 tb[ETHTOOL_A_PREEMPT_QUEUES_PREEMPTIBLE], &mod);
		mod = true;
	}

	ret = 0;
	if (!mod)
		goto out_ops;

	ret = dev->ethtool_ops->set_preempt(dev, &preempt);
	if (ret < 0)
		GENL_SET_ERR_MSG(info, "frame preemption settings update failed");
	else
		ethtool_notify(dev, ETHTOOL_MSG_PREEMPT_NTF, NULL);

out_ops:
	ethnl_ops_complete(dev);
out_rtnl:
	rtnl_unlock();
out_dev:
	dev_put(dev);
	return ret;
}
