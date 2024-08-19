#include "pch.h"
#include "src/message_queue.h"

class MessageQueueTests : public testing::Test
{
protected:
	MessageQueueTests()
	{}

	MessageQueue queue;
};

TEST_F(MessageQueueTests, QueueBeginsEmpty)
{
	EXPECT_EQ(0, queue.size());
}

TEST_F(MessageQueueTests, EnqueuingAMessageIncreasesQueueSize)
{
	Transaction new_transaction;
	queue.enqueue(new_transaction);
	EXPECT_EQ(1, queue.size());
}

TEST_F(MessageQueueTests, DequeuingAMessageReducesSizeAndReturnsSameTransaction)
{
	uint8_t data[1] = { 3 };
	Transaction new_transaction;
	new_transaction.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(new_transaction);
	Transaction* dequeued_transaction = queue.dequeue();
	EXPECT_EQ(3, dequeued_transaction->get_tx_data()[0]);
	EXPECT_EQ(0, queue.size());
}

TEST_F(MessageQueueTests, ResettingQueueSetsSizeToZero)
{
	Transaction new_transaction;
	queue.enqueue(new_transaction);
	queue.reset();
	EXPECT_EQ(0, queue.size());
}

TEST_F(MessageQueueTests, NewlyQueuedMessageStartsNotReady)
{
	Transaction new_transaction;
	queue.enqueue(new_transaction);
	EXPECT_FALSE(queue.is_response_ready());
}

TEST_F(MessageQueueTests, IfTransactionIsMarkedFinishedMessageIsReady)
{
	Transaction new_transaction;
	queue.enqueue(new_transaction);
	Transaction* queued_transaction = queue.get_active_transaction();
	queued_transaction->mark_finished();
	EXPECT_TRUE(queue.is_response_ready());
}

TEST_F(MessageQueueTests, TwoEnqueuedMessagesDequeueInFIFOOrder)
{
	uint8_t data[1] = { 3 };
	Transaction transaction_1;
	transaction_1.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_1);

	data[0] = { 5 };
	Transaction transaction_2;
	transaction_2.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_2);

	Transaction* dequeued_transaction_1 = queue.dequeue();
	EXPECT_EQ(3, dequeued_transaction_1->get_tx_data()[0]);

	Transaction* dequeued_transaction_2 = queue.dequeue();
	EXPECT_EQ(5, dequeued_transaction_2->get_tx_data()[0]);
}

// ===============================BAD BEHAVIOUR TESTS================================

TEST_F(MessageQueueTests, GetActiveTransactionUpdatesAsMessagesAreDequeuedANDIncrementActiveIndexIsCalled)
{
	uint8_t data[1] = { 3 };
	Transaction transaction_1;
	transaction_1.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_1);

	data[0] = { 5 };
	Transaction transaction_2;
	transaction_2.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_2);

	queue.dequeue();

	queue.increment_active_index_if_finished();

	Transaction* active_transaction = queue.get_active_transaction();
	EXPECT_EQ(5, active_transaction->get_tx_data()[0]);
}

TEST_F(MessageQueueTests, IfMessageIsDequeuedButAvailableToSendIsntCalledActiveTransactionDoesntMove)
{
	uint8_t data[1] = { 3 };
	Transaction transaction_1;
	transaction_1.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_1);

	data[0] = { 5 };
	Transaction transaction_2;
	transaction_2.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_2);

	queue.dequeue();

	Transaction* active_transaction = queue.get_active_transaction();
	EXPECT_EQ(3, active_transaction->get_tx_data()[0]); //TODO: This should fail (Equal 5)
}

TEST_F(MessageQueueTests, TransactionBeingMarkedFinishedCausesActiveTransactionToMoveForward)
{
	uint8_t data[1] = { 3 };
	Transaction transaction_1;
	transaction_1.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_1);

	data[0] = { 5 };
	Transaction transaction_2;
	transaction_2.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_2);

	Transaction* active_transaction = queue.get_active_transaction();
	EXPECT_EQ(3, active_transaction->get_tx_data()[0]);

	active_transaction->mark_finished();

	queue.increment_active_index_if_finished();

	active_transaction = queue.get_active_transaction();
	EXPECT_EQ(5, active_transaction->get_tx_data()[0]);
}

TEST_F(MessageQueueTests, TransactionBeingMarkedFinishedDoesntCauseActiveTransactionToMoveForwardIfIncrementActiveIndexIsNotCalled)
{
	uint8_t data[1] = { 3 };
	Transaction transaction_1;
	transaction_1.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_1);

	data[0] = { 5 };
	Transaction transaction_2;
	transaction_2.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_2);

	Transaction* active_transaction = queue.get_active_transaction();
	EXPECT_EQ(3, active_transaction->get_tx_data()[0]);

	active_transaction->mark_finished();

	active_transaction = queue.get_active_transaction();
	EXPECT_EQ(3, active_transaction->get_tx_data()[0]);
}

// ============================END BAD BEHAVIOUR TESTS===============================

TEST_F(MessageQueueTests, MessageIsImmediatelyAvailableToSendAfterBeingEnqueued)
{
	uint8_t data[1] = { 3 };
	Transaction transaction_1;
	transaction_1.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_1);

	EXPECT_TRUE(queue.available_to_send());
}

TEST_F(MessageQueueTests, MessageIsNotAvailableToSendIfMarkedSent)
{
	uint8_t data[1] = { 3 };
	Transaction transaction_1;
	transaction_1.load_transmission_data(1, 3, data, 1, 5);
	queue.enqueue(transaction_1);

	Transaction* active_transaction = queue.get_active_transaction();
	active_transaction->mark_sent();

	EXPECT_FALSE(queue.available_to_send());
}